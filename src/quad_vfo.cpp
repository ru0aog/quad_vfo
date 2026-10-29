// quad_vfo.cpp  
// ============================================================================  
// Автономный квадратурный LO: порт quad_* из src/vfo/soft_rx.cpp (RU0AOG)  
// без зависимостей от vfo/pins/soft_tx/mem_arena.  
//  
// PIO: 4-тактовая программа на выбранном PIO и 2 соседних пинах.  
// LO = clk_sys/(4*div), div — делитель SM в Q8.  
// start: LO = freq - cfg->if_shift_hz, сканер PLL по взвешенной сумме  
// штрафов П2..П5 с запретной зоной П1 и fallback "наименьшее зло" П6,  
// перепрошивка clk_sys через quad_vfo_set_clk_sys(), делитель от  
// реальной clock_get_hz(clk_sys).  
//  
// Жизненный цикл: offset программы хранится per-PIO; повторный init  
// транзакционен — валидация и проверка доступности SM до освобождения  
// прежних ресурсов. Отказы после "точки невозврата" приводят состояние  
// в согласованный вид через fail_path().  
// ============================================================================  
#include "quad_vfo.h"  
#include "hardware/pio.h"  
#include "hardware/clocks.h"  
#include "hardware/vreg.h"  
  
// Аппаратные ограничения pll_sys (datasheet): VCO 400..1600 МГц.  
// Это НЕ пользовательский предел pll_vco_max_hz — он проверяется  
// в init отдельно и может быть уже, но не шире.  
#define QUAD_VCO_HW_MIN_HZ 400000000ULL  
#define QUAD_VCO_HW_MAX_HZ 1600000000ULL  
// Нижняя граница VCO в сканере: 750 МГц — рабочее окно исходника  
// (не аппаратный предел), оставлено как было в soft_rx.  
  
// ---------------------------------------------------------------------------  
// PIO-программа квадратурного LO: 4 инструкции, set pins n.  
// bit0 -> pin_i (I), bit1 -> pin_i+1 (Q).  
// Фазы: 0b01 -> 0b11 -> 0b10 -> 0b00  (I:1,1,0,0  Q:0,1,1,0,  
// Q отстаёт от I на 90°). Коды литеральные: set = 0xE000|n  
// (pio_encode_set в SDK — не constexpr, в инициализаторе нельзя).  
static const uint16_t pio_quad_instructions[4] = {  
  0xE001,  // set pins, 0b01  I=1 Q=0  
  0xE003,  // set pins, 0b11  I=1 Q=1  
  0xE002,  // set pins, 0b10  I=0 Q=1  
  0xE000,  // set pins, 0b00  I=0 Q=0  
};  
static const pio_program_t pio_quad_program = {  
  .instructions = pio_quad_instructions,  
  .length       = 4,  
  .origin       = -1,  
};  
  
// ---- Состояние -------------------------------------------------------------  
static quad_vfo_cfg_t s_cfg;  
static bool     s_have_cfg   = false;  
static unsigned s_sm         = 0;  
static bool     s_sm_claimed = false;  
static bool     s_active     = false;  
static uint32_t s_lo_actual_hz = 0;  
static int32_t  s_lo_error_mhz = 0;  
  
// Программа загружается в каждый PIO-блок отдельно.  
static int      s_offset[2]   = { -1, -1 };   // по индексу pio_get_index()  
static unsigned s_cur_offset  = 0;  
  
// ---------------------------------------------------------------------------  
void quad_vfo_default_cfg(quad_vfo_cfg_t *cfg) {  
  memset(cfg, 0, sizeof(*cfg));  
  cfg->pio   = pio1;  
  cfg->sm    = -1;  
  cfg->pin_i = 0;  
  
  // Номинал 133 МГц: VCO = 12M*133 = 1596 МГц, /12 = 133 МГц.  
  cfg->pll_nominal = { 133, 6, 2, 133000000u,  
                       (uint32_t)VREG_VOLTAGE_DEFAULT, false, 1 };  
  // Потолок, доказанный OCTEST в исходнике: 380 МГц @ VREG_1_30.  
  // СВЕРИТЬ с инициализатором pll_ceiling в вашем soft_tx.cpp.  
  cfg->pll_ceiling = { 95, 3, 1, 380000000u,  
                       (uint32_t)VREG_VOLTAGE_1_30, true, 1 };  
  
  cfg->xosc_hz            = 12000000ULL;  
  cfg->clk_sys_min_hz     = 240000000ULL;  
  cfg->clk_sys_max_hz     = 133000000ULL;   // "не доказано" -> pll_ceiling  
  cfg->pll_vco_max_hz     = QUAD_VCO_HW_MAX_HZ;  
  cfg->clk_sys_nominal_hz = 133000000ULL;  
  
  cfg->if_shift_hz    = 11000u;  
  cfg->nco_max_hz     = 62500u;  
  cfg->nco_sweet_lo   = 9600u;  
  cfg->nco_sweet_hi   = 14500u;  
  cfg->nco_forbid_lo  = 0u;  
  cfg->nco_forbid_hi  = 5000u;  
}  
  
// ---------------------------------------------------------------------------  
static bool pio_ok(PIO p) { return p == pio0 || p == pio1; }  
  
// Валидация одного PLL-профиля: диапазоны полей + аппаратное окно VCO.  
static bool pll_profile_ok(const quad_vfo_pll_t *p, uint64_t xosc_hz) {  
  if (p->refdiv < 1 || p->refdiv > 63)              return false;  
  if (p->p1 < 2 || p->p1 > 7)                       return false;  
  if (p->p2 < 1 || p->p2 > 7)                       return false;  
  if (p->fbdiv < 16 || p->fbdiv > 320)              return false;  
  if (p->clk_sys_hz == 0)                           return false;  
  uint64_t vco = (xosc_hz / p->refdiv) * p->fbdiv;  
  if (vco < QUAD_VCO_HW_MIN_HZ || vco > QUAD_VCO_HW_MAX_HZ)  
    return false;                                 // аппаратный диапазон  
  return true;  
}  
  
// Согласованность: вычисленная clk = xosc*fbdiv/(refdiv*p1*p2) должна  
// совпадать с заявленной clk_sys_hz в пределах 1% (округление SDK).  
static bool pll_profile_consistent(const quad_vfo_pll_t *p,  
                                   uint64_t xosc_hz) {  
  uint64_t calc = ((uint64_t)p->fbdiv * xosc_hz)  
                  / (uint64_t)(p->refdiv * p->p1 * p->p2);  
  uint64_t want = p->clk_sys_hz;  
  uint64_t diff = (calc > want) ? (calc - want) : (want - calc);  
  return diff <= want / 100u;  
}  
  
// ---------------------------------------------------------------------------  
// Единый путь отказа после "точки невозврата": генерация уже прервана —  
// приводим железо и состояние в согласованный вид и возвращаем false.  
static bool fail_path(void) {  
  pio_sm_set_enabled(s_cfg.pio, s_sm, false);  
  pio_sm_set_consecutive_pindirs(s_cfg.pio, s_sm, s_cfg.pin_i, 2, false);  
  gpio_set_dir(s_cfg.pin_i,     GPIO_IN);  
  gpio_set_dir(s_cfg.pin_i + 1, GPIO_IN);  
  s_active       = false;  
  s_lo_actual_hz = 0;  
  s_lo_error_mhz = 0;  
  return false;  
}  
  
// ---------------------------------------------------------------------------  
bool quad_vfo_init(const quad_vfo_cfg_t *cfg) {  
  // === Фаза 1: валидация новой конфигурации ===  
  if (!cfg)                                         return false;  
  if (!pio_ok(cfg->pio))                            return false;  
  if (cfg->pin_i + 1 > 29)                          return false;  
  if (cfg->sm < -1 || cfg->sm > 3)                  return false;  
  if (cfg->xosc_hz == 0)                            return false;  
  if (cfg->clk_sys_min_hz == 0)                     return false;  
  if (cfg->nco_max_hz == 0)                         return false;  
  if (cfg->nco_sweet_lo  > cfg->nco_sweet_hi)       return false;  
  if (cfg->nco_forbid_lo > cfg->nco_forbid_hi)      return false;  
  if (cfg->if_shift_hz  >= cfg->nco_max_hz)         return false;  
  if (cfg->pll_vco_max_hz == 0 ||  
      cfg->pll_vco_max_hz > QUAD_VCO_HW_MAX_HZ)     return false;  
  if (!pll_profile_ok(&cfg->pll_nominal, cfg->xosc_hz))  return false;  
  if (!pll_profile_ok(&cfg->pll_ceiling, cfg->xosc_hz))  return false;  
  if (!pll_profile_consistent(&cfg->pll_nominal, cfg->xosc_hz))  
    return false;  
  if (!pll_profile_consistent(&cfg->pll_ceiling, cfg->xosc_hz))  
    return false;  
  
  // === Фаза 2: проверка доступности ресурсов ДО освобождения старых ===  
  // Своя уже захваченная SM — легальный кандидат на переиспользование.  
  bool same_pio = (s_have_cfg && s_sm_claimed && s_cfg.pio == cfg->pio);  
  int  new_sm   = -1;  
  
  if (cfg->sm >= 0) {  
    bool mine = (same_pio && s_sm == (unsigned)cfg->sm);  
    if (!mine && pio_sm_is_claimed(cfg->pio, (uint)cfg->sm))  
      return false;                              // чужая — отказ,  
                                                 // старое не тронуто  
    new_sm = cfg->sm;  
  } else {  
    // Автовыбор БЕЗ паники SDK: ищем свободную SM заранее.  
    for (unsigned cand = 0; cand < 4; cand++) {  
      bool mine = (same_pio && s_sm == cand);  
      if (mine || !pio_sm_is_claimed(cfg->pio, cand)) {  
        new_sm = (int)cand;  
        break;  
      }  
    }  
    if (new_sm < 0) return false;                // нет свободных SM —  
                                                 // старое сохранено  
  }  
  
  // === Фаза 3: освобождение прежних ресурсов ===  
  if (s_sm_claimed) {  
    bool same_sm = (same_pio && s_sm == (unsigned)new_sm);  
    if (!same_sm) {  
      // Другая SM/PIO — гасим и освобождаем старую.  
      quad_vfo_stop();  
      pio_sm_unclaim(s_cfg.pio, s_sm);  
      s_sm_claimed = false;  
    } else {  
      // Та же SM — генерацию останавливаем, claim сохраняем.  
      if (s_active) quad_vfo_stop();  
    }  
  }  
  
  // === Фаза 4: захват новых ресурсов ===  
  s_cfg      = *cfg;  
  s_have_cfg = true;  
  s_sm       = (unsigned)new_sm;  
  
  int idx = (int)pio_get_index(s_cfg.pio);  
  if (s_offset[idx] < 0) {  
    int off = pio_add_program(s_cfg.pio, &pio_quad_program);  
    if (off < 0) {                               // память инструкций  
      s_have_cfg = false;                        // кончилась — откат  
      return false;                              // состояния  
    }  
    s_offset[idx] = off;  
  }  
  s_cur_offset = (unsigned)s_offset[idx];  
  
  if (!s_sm_claimed) {  
    pio_sm_claim(s_cfg.pio, s_sm);  
    s_sm_claimed = true;  
  }  
  return true;  
}  
  
// ---------------------------------------------------------------------------  
bool quad_vfo_init_simple(PIO pio, uint8_t pin_i) {  
  quad_vfo_cfg_t cfg;  
  quad_vfo_default_cfg(&cfg);  
  cfg.pio   = pio;  
  cfg.pin_i = pin_i;  
  return quad_vfo_init(&cfg);  
}  
  
// ---------------------------------------------------------------------------  
bool quad_vfo_start(uint32_t freq_hz) {  
  if (!s_have_cfg) return false;  
  // Ранний отказ БЕЗ вмешательства в работающий генератор.  
  if (freq_hz <= s_cfg.if_shift_hz) return false;  
  
  const uint64_t xosc_hz = s_cfg.xosc_hz;  
  uint32_t lo_hz = freq_hz - s_cfg.if_shift_hz;  
  Serial.printf("[QUAD] Целевая частота LO %lu\n", (unsigned long)lo_hz);  
  
  // === Подгонка clk_sys под ЦЕЛЫЙ делитель LO ===  
  // LO = clk_sys/(4*D). Целый D — без джиттера дробного делителя PIO;  
  // дробный — компромисс со штрафом. На фиксированных 133 МГц целый D  
  // на ВЧ слишком крупный — ищем достижимую PLL-комбинацию.  
  
  // Потолок скана = min(паспорт, аппаратный барьер). Паспорт "не  
  // доказан" (окно пустое) -> потолок из pll_ceiling.  
  uint64_t clk_max = (s_cfg.clk_sys_max_hz < 420000000ULL)  
                     ? s_cfg.clk_sys_max_hz : 420000000ULL;  
  if (clk_max < s_cfg.clk_sys_min_hz) {  
    clk_max = s_cfg.pll_ceiling.clk_sys_hz;  
    Serial.printf("[QUAD] паспорт clk_max=%lu МГц не доказан — "  
                  "потолок из pll_ceiling=%lu МГц\n",  
                  (unsigned long)(s_cfg.clk_sys_max_hz / 1000000UL),  
                  (unsigned long)(clk_max / 1000000ULL));  
  }  
  const uint64_t vco_max = (s_cfg.pll_vco_max_hz < QUAD_VCO_HW_MAX_HZ)  
                           ? s_cfg.pll_vco_max_hz : QUAD_VCO_HW_MAX_HZ;  
  
  // Взвешенная сумма штрафов: W2 >> W3 >> W4 >> W5.  
  const uint64_t W2 = 1000000000ull;  // удаление от сладкой зоны (за 1 Гц)  
  const uint64_t W3 = 1000000ull;     // штраф за дробный делитель  
  const uint64_t W4 = 1000ull;        // близость дроби к k/8  
  const uint64_t W5 = 1ull;           // шина ниже максимума (за 1 МГц)  
  
  quad_vfo_pll_t best        = s_cfg.pll_nominal;  
  uint64_t  best_cost   = UINT64_MAX;  
  uint32_t  best_div_q8 = 0;  
  bool      best_is_int = true;  
  bool      best_legal  = false;  
  
  // Fallback "наименьшее зло": лучший среди НЕлегальных (дырка в зоне).  
  quad_vfo_pll_t fb_best   = s_cfg.pll_nominal;  
  uint64_t  fb_bad    = UINT64_MAX;  
  uint32_t  fb_div_q8 = 0;  
  bool      fb_is_int = true;  
  
  for (uint32_t refdiv = 1; refdiv <= 3; refdiv++) {  
    uint64_t ref_hz = xosc_hz / refdiv;  
    for (uint32_t p1 = 2; p1 <= 7; p1++) {  
      for (uint32_t p2 = 1; p2 <= 7; p2++) {  
        if (p1 < p2) continue;  
        uint32_t pdiv = p1 * p2;  
        for (uint32_t fbdiv = 16; fbdiv <= 320; fbdiv++) {  
          uint64_t vco = ref_hz * fbdiv;  
          if (vco < 750000000ULL || vco > vco_max) continue; // 750M —  
          // рабочее окно исходника (не аппаратный предел 400M)  
          uint64_t clk = vco / pdiv;  
          if (clk < s_cfg.clk_sys_min_hz || clk > clk_max) continue;  
  
          for (int frac_mode = 0; frac_mode < 2; frac_mode++) {  
            uint32_t div_q8;  
            if (frac_mode == 0) {                       // целый  
              uint64_t D = (clk + 2ull * lo_hz) / (4ull * lo_hz);  
              if (D < 2)      D = 2;  
              if (D > 0xFFFF) D = 0xFFFF;  
              div_q8 = (uint32_t)(D << 8);  
            } else {                                    // дробный Q8  
              uint64_t q = (clk * 256ull + 2ull * lo_hz) / (4ull * lo_hz);  
              if (q < (2u << 8))      q = 2u << 8;  
              if (q > (0xFFFFu << 8)) q = 0xFFFFu << 8;  
              div_q8 = (uint32_t)q;  
            }  
            uint64_t lo_got = (clk * 256ull) / (4ull * (uint64_t)div_q8);  
            if (lo_got >= freq_hz) continue;  
            int64_t f_nco = (int64_t)freq_hz - (int64_t)lo_got;  
            if (f_nco <= 0 || f_nco >= (int64_t)s_cfg.nco_max_hz) continue;  
  
            uint32_t frac   = div_q8 & 0xFFu;  
            bool     is_int = (frac == 0);  
  
            // --- П1: жёсткий запрет дырки [forbid_lo..forbid_hi] ---  
            if (f_nco >= (int64_t)s_cfg.nco_forbid_lo &&  
                f_nco <= (int64_t)s_cfg.nco_forbid_hi) {  
              uint64_t bad = (uint64_t)((int64_t)s_cfg.nco_forbid_hi - f_nco);  
              if (bad < fb_bad) {  
                fb_bad = bad; fb_div_q8 = div_q8; fb_is_int = is_int;  
                fb_best.refdiv=refdiv; fb_best.p1=p1; fb_best.p2=p2;  
                fb_best.fbdiv=fbdiv;  fb_best.clk_sys_hz=clk;  
                fb_best.vsel=quad_vfo_vsel_for(clk);  
                fb_best.is_oc=(clk > s_cfg.clk_sys_nominal_hz);  
              }  
              continue;  
            }  
  
            // --- стоимость легального кандидата ---  
            uint64_t cost = 0;  
            if      (f_nco < (int64_t)s_cfg.nco_sweet_lo)  
              cost += (uint64_t)((int64_t)s_cfg.nco_sweet_lo - f_nco) * W2;  
            else if (f_nco > (int64_t)s_cfg.nco_sweet_hi)  
              cost += (uint64_t)(f_nco - (int64_t)s_cfg.nco_sweet_hi) * W2;  
            if (!is_int) cost += W3;  
            if (!is_int) {  
              int32_t m    = (int32_t)frac % 32;  
              int32_t dist = (m < 16) ? m : 32 - m;  
              cost += (uint64_t)(16 - dist) * W4;  
            }  
            cost += ((clk_max - clk) / 1000000ull) * W5;  
  
            if (cost < best_cost) {  
              best_cost   = cost;  
              best_div_q8 = div_q8;  
              best_is_int = is_int;  
              best_legal  = true;  
              best.refdiv=refdiv; best.p1=p1; best.p2=p2;  
              best.fbdiv=fbdiv;   best.clk_sys_hz=clk;  
              best.vsel=quad_vfo_vsel_for(clk);  
              best.is_oc=(clk > s_cfg.clk_sys_nominal_hz);  
            }  
          }  
        }  
      }  
    }  
  }  
  
  // П6: пустой скан — ранний отказ, работающий генератор НЕ трогаем.  
  if (!best_legal && fb_bad == UINT64_MAX) {  
    Serial.println("[QUAD] ОШИБКА: скан не дал ни одного кандидата — "  
                   "проверьте clk_sys_max/pll_ceiling. LO не тронут.");  
    return false;  
  }  
  if (!best_legal) {  
    Serial.println("[QUAD] ВНИМАНИЕ: нет кандидата с дыркой вне "  
                   "запретной зоны — ставлю наименьшее зло!");  
    best        = fb_best;  
    best_div_q8 = fb_div_q8;  
    best_is_int = fb_is_int;  
  }  
  
  // === Точка невозврата: дальше любой отказ -> fail_path() ===  
  pio_sm_set_enabled(s_cfg.pio, s_sm, false);  
  s_active = false;  
  
  // Перепрошиваем выбранную шину. Результат ПРОВЕРЯЕМ.  
  uint32_t vsel = best.vsel;  
  if (vsel == 0 || vsel < (uint32_t)VREG_VOLTAGE_1_10)  
    vsel = quad_vfo_vsel_for(best.clk_sys_hz);  
  bool pll_ok = quad_vfo_set_clk_sys(&best, vsel,  
                                     &s_cfg.pll_nominal, xosc_hz);  
  if (!pll_ok)  
    Serial.println("[QUAD] ВНИМАНИЕ: clk_sys не подтверждён — "  
                   "делитель считаем от фактической шины");  
  
  // Делитель пересчитываем от РЕАЛЬНОЙ шины (кэш частоты может  
  // отставать — в исходнике это давало уход LO на 2.8 кГц).  
  uint64_t clk_sys_hz = clock_get_hz(clk_sys);  
  uint32_t div_q8;  
  if (best_is_int) {  
    uint64_t D = (clk_sys_hz + 2ull * lo_hz) / (4ull * lo_hz);  
    if (D < 2)      D = 2;  
    if (D > 0xFFFF) D = 0xFFFF;  
    div_q8 = (uint32_t)(D << 8);  
  } else {  
    uint64_t q = (clk_sys_hz * 256ull + 2ull * lo_hz) / (4ull * lo_hz);  
    if (q < (2u << 8))      q = 2u << 8;  
    if (q > (0xFFFFu << 8)) q = 0xFFFFu << 8;  
    div_q8 = (uint32_t)q;  
  }  
  
  // Перепроверка NCO-ограничений на фактической шине — сканер считал  
  // от плановой, а реальная могла остаться номинальной после отказа.  
  uint64_t lo_got = (clk_sys_hz * 256ull) / (4ull * (uint64_t)div_q8);  
  int64_t  f_nco  = (int64_t)freq_hz - (int64_t)lo_got;  
  bool nco_ok = (lo_got < freq_hz) &&  
                (f_nco > 0) &&  
                (f_nco < (int64_t)s_cfg.nco_max_hz) &&  
                !(f_nco >= (int64_t)s_cfg.nco_forbid_lo &&  
                  f_nco <= (int64_t)s_cfg.nco_forbid_hi);  
  if (!nco_ok) {  
    Serial.printf("[QUAD] ОШИБКА: на фактической шине %lu кГц "  
                  "f_nco=%ld вне допустимых зон — запуск отменён\n",  
                  (unsigned long)(clk_sys_hz / 1000ULL), (long)f_nco);  
    return fail_path();  
  }  
  
  // Фактическая частота LO и расчётная ошибка (мГц) — по принятой  
  // частоте шины, не по независимому измерению.  
  uint64_t lo_mhz = (clk_sys_hz * 256ull * 1000ull) /  
                    (4ull * (uint64_t)div_q8);  
  s_lo_actual_hz = (uint32_t)(lo_mhz / 1000ull);  
  s_lo_error_mhz =  
      (int32_t)((int64_t)lo_hz * 1000ll - (int64_t)lo_mhz);  
  
  {  
    uint32_t frac = div_q8 & 0xFFu;  
    bool sweet = (f_nco >= (int64_t)s_cfg.nco_sweet_lo &&  
                  f_nco <= (int64_t)s_cfg.nco_sweet_hi);  
    Serial.printf("[QUAD] clkdiv=%lu.%03lu (%s)  lo=%lu Гц "  
                  "err=%ld мГц  f_nco=%ld Гц sweet=%d\n",  
                  (unsigned long)(div_q8 >> 8), (unsigned long)frac,  
                  frac ? "дробный" : "целый",  
                  (unsigned long)s_lo_actual_hz,  
                  (long)s_lo_error_mhz,  
                  (long)f_nco, sweet ? 1 : 0);  
  }  
  
  // === Конфигурация SM под квадратуру ===  
  pio_sm_config cq = pio_get_default_sm_config();  
  sm_config_set_wrap(&cq, s_cur_offset + 0, s_cur_offset + 3);  
  sm_config_set_set_pins(&cq, s_cfg.pin_i, 2);   // I=base, Q=base+1  
  sm_config_set_clkdiv(&cq, (float)div_q8 / 256.0f);  
  pio_gpio_init(s_cfg.pio, s_cfg.pin_i);  
  pio_gpio_init(s_cfg.pio, s_cfg.pin_i + 1);  
  
  pio_sm_init(s_cfg.pio, s_sm, s_cur_offset, &cq);  
  // Направление ПОСЛЕДНИМ, после sm_init.  
  pio_sm_set_consecutive_pindirs(s_cfg.pio, s_sm, s_cfg.pin_i, 2, true);  
  pio_sm_restart(s_cfg.pio, s_sm);  
  pio_sm_set_enabled(s_cfg.pio, s_sm, true);  
  
  s_active = true;  
  return true;  
}  
  
// ---------------------------------------------------------------------------  
void quad_vfo_stop(void) {  
  if (!s_have_cfg) return;  
  pio_sm_set_enabled(s_cfg.pio, s_sm, false);  
  pio_sm_set_consecutive_pindirs(s_cfg.pio, s_sm, s_cfg.pin_i, 2, false);  
  gpio_set_dir(s_cfg.pin_i,     GPIO_IN);  
  gpio_set_dir(s_cfg.pin_i + 1, GPIO_IN);  
  s_active       = false;  
  s_lo_actual_hz = 0;  
  s_lo_error_mhz = 0;  
}  
  
uint32_t quad_vfo_actual_hz(void) { return s_lo_actual_hz; }  
int32_t  quad_vfo_error_mhz(void) { return s_lo_error_mhz; }  
bool     quad_vfo_active(void)    { return s_active; }  
  
// ---------------------------------------------------------------------------  
void quad_vfo_debug(void) {  
  if (!s_have_cfg) { Serial.println("[LODBG] не инициализирован"); return; }  
  uint32_t meas_clk = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS) * 1000u;  
  uint32_t cdiv     = s_cfg.pio->sm[s_sm].clkdiv;  
  
  Serial.printf("[LODBG] clk_sys: cached=%lu kHz  measured=%lu kHz\n",  
                (unsigned long)(clock_get_hz(clk_sys) / 1000ULL),  
                (unsigned long)(meas_clk / 1000u));  
  Serial.printf("[LODBG] PIO=%d SM=%d offset=%u  clkdiv=0x%08lX "  
                "(int=%lu frac=%lu)\n",  
                pio_get_index(s_cfg.pio), (int)s_sm, s_cur_offset,  
                (unsigned long)cdiv,  
                (unsigned long)(cdiv >> 16),  
                (unsigned long)((cdiv >> 8) & 0xFF));  
  Serial.printf("[LODBG] SM ctrl=0x%08lX enabled=%d  execctrl=0x%08lX\n",  
                (unsigned long)s_cfg.pio->ctrl,  
                (int)((s_cfg.pio->ctrl & (1u << s_sm)) != 0),  
                (unsigned long)s_cfg.pio->sm[s_sm].execctrl);  
  Serial.printf("[LODBG] lo_hz=%lu  err=%ld мГц  active=%d\n",  
                (unsigned long)s_lo_actual_hz, (long)s_lo_error_mhz,  
                (int)s_active);  
}