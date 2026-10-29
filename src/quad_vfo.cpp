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
// Режим точной частоты (nco_max_hz == 0): LO = freq, все NCO-зоны  
// выключены, сканер выбирает делитель по минимуму |f_nco|.  
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
  
// Допустимая |f_nco| в режиме точной частоты при пересчёте делителя  
// после отката шины (recompute_div): физически ошибка ограничена  
// дискретностью делителя, 1 МГц — широкий задел.  
#define QUAD_EXACT_MODE_TOL_HZ 1000000  
  
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
  // PLL-подсистема  
  cfg->pll_nominal = { 133, 6, 2, 133000000u,  
                       (uint32_t)VREG_VOLTAGE_DEFAULT, false, 1 };  
  cfg->pll_ceiling = { 95, 3, 1, 380000000u,  
                       (uint32_t)VREG_VOLTAGE_1_30, true, 1 };  
  cfg->xosc_hz           = 12000000ULL;  
  cfg->clk_sys_min_hz    = 240000000ULL;  
  cfg->clk_sys_max_hz    = 420000000ULL;  
  cfg->pll_vco_max_hz    = QUAD_VCO_HW_MAX_HZ;  
  cfg->clk_sys_nominal_hz= 133000000ULL;  
  // ПЧ и NCO-зоны (режим приёмника, как в soft_rx)  
  cfg->if_shift_hz    = 11000u;  
  cfg->nco_max_hz     = 62500u;  
  cfg->nco_sweet_lo   = 9600u;  
  cfg->nco_sweet_hi   = 14500u;  
  cfg->nco_forbid_lo  = 0u;  
  cfg->nco_forbid_hi  = 5000u;  
}  
  
// ---------------------------------------------------------------------------  
static bool pio_ok(PIO p) { return p == pio0 || p == pio1; }  
  
// Аппаратные ограничения профиля: делители в диапазонах SDK,  
// clk_sys_hz != 0, VCO в аппаратном окне 400..1600 МГц.  
static bool pll_profile_ok(const quad_vfo_pll_t *p, uint64_t xosc_hz) {  
  if (p->refdiv < 1 || p->refdiv > 63)              return false;  
  if (p->p1 < 2 || p->p1 > 7)                       return false;  
  if (p->p2 < 1 || p->p2 > 7)                       return false;  
  if (p->fbdiv < 16 || p->fbdiv > 320)              return false;  
  if (p->clk_sys_hz == 0)                           return false;  
  uint64_t vco = (xosc_hz / p->refdiv) * p->fbdiv;  
  if (vco < QUAD_VCO_HW_MIN_HZ || vco > QUAD_VCO_HW_MAX_HZ)  
    return false;  
  return true;  
}  
  
// Согласованность полей: вычисленная частота == clk_sys_hz (±1%).  
static bool pll_profile_consistent(const quad_vfo_pll_t *p, uint64_t xosc_hz) {  
  uint64_t calc = ((xosc_hz / p->refdiv) * p->fbdiv) / (p->p1 * p->p2);  
  uint64_t diff = (calc > p->clk_sys_hz)  
                  ? calc - p->clk_sys_hz : p->clk_sys_hz - calc;  
  return diff <= p->clk_sys_hz / 100u;  
}  
  
// Единый путь отказа после "точки невозврата" (SM уже погашена):  
// состояние приводится к согласованному виду, пины в input.  
static void fail_path(void) {  
  pio_sm_set_enabled(s_cfg.pio, s_sm, false);  
  pio_sm_set_consecutive_pindirs(s_cfg.pio, s_sm, s_cfg.pin_i, 2, false);  
  gpio_set_dir(s_cfg.pin_i,     GPIO_IN);  
  gpio_set_dir(s_cfg.pin_i + 1, GPIO_IN);  
  s_active       = false;  
  s_lo_actual_hz = 0;  
  s_lo_error_mhz = 0;  
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
  
  // nco_max_hz == 0 легален: режим точной частоты (init_simple).  
  // Тогда ВСЕ NCO-зоны обязаны быть нулевыми — иначе cfg противоречив.  
  if (cfg->nco_max_hz == 0 &&  
      (cfg->nco_sweet_lo  != 0 || cfg->nco_sweet_hi  != 0 ||  
       cfg->nco_forbid_lo != 0 || cfg->nco_forbid_hi != 0))  
                                                    return false;  
  if (cfg->nco_sweet_lo  > cfg->nco_sweet_hi)       return false;  
  if (cfg->nco_forbid_lo > cfg->nco_forbid_hi)      return false;  
  // Точный режим несовместим с ПЧ-сдвигом: стоимость считается от  
  // freq, а не от freq-shift — сдвиг игнорировался бы.  
  if (cfg->nco_max_hz == 0 && cfg->if_shift_hz != 0) return false;  
  // ПЧ-проверка только в режиме приёмника (nco_max != 0).  
  if (cfg->nco_max_hz != 0 &&  
      cfg->if_shift_hz >= cfg->nco_max_hz)          return false;  
  
  if (cfg->pll_vco_max_hz == 0 ||  
      cfg->pll_vco_max_hz > QUAD_VCO_HW_MAX_HZ)     return false;  
  if (!pll_profile_ok(&cfg->pll_nominal, cfg->xosc_hz))  return false;  
  if (!pll_profile_ok(&cfg->pll_ceiling, cfg->xosc_hz))  return false;  
  if (!pll_profile_consistent(&cfg->pll_nominal, cfg->xosc_hz))  
    return false;  
  if (!pll_profile_consistent(&cfg->pll_ceiling, cfg->xosc_hz))  
    return false;  
  
  // === Фаза 2: проверка доступности ресурсов ДО освобождения старых ===  
  bool same_sm = (s_sm_claimed && s_cfg.pio == cfg->pio &&  
                  cfg->sm >= 0 && (unsigned)cfg->sm == s_sm);  
  int target_sm = -1;  
  if (cfg->sm >= 0) {  
    if (!same_sm && pio_sm_is_claimed(cfg->pio, (uint)cfg->sm))  
      return false;  
    target_sm = cfg->sm;  
  } else {  
    // Автовыбор: предвыбираем SM до освобождения старой, чтобы не  
    // попасть в pio_claim_unused_sm(required=true) -> panic.  
    for (int cand = 0; cand < 4; cand++) {  
      bool mine = (s_sm_claimed && s_cfg.pio == cfg->pio &&  
                   (int)s_sm == cand);  
      if (mine || !pio_sm_is_claimed(cfg->pio, (uint)cand)) {  
        target_sm = cand;  
        break;  
      }  
    }  
    if (target_sm < 0) return false;   // свободных SM нет — старое сохранено  
  }  
  
  // === Фаза 3: освобождение прежних ресурсов ===  
  if (s_sm_claimed) {  
    quad_vfo_stop();  
    pio_sm_unclaim(s_cfg.pio, s_sm);  
    s_sm_claimed = false;  
  }  
  
  // === Фаза 4: захват новых ресурсов ===  
  s_cfg      = *cfg;  
  s_have_cfg = true;  
  
  int idx = (int)pio_get_index(s_cfg.pio);  
  if (s_offset[idx] < 0) {  
    int off = pio_add_program(s_cfg.pio, &pio_quad_program);  
    if (off < 0) {                     // память инструкций исчерпана  
      s_have_cfg = false;              // состояние: не инициализированы  
      return false;  
    }  
    s_offset[idx] = off;  
  }  
  s_cur_offset = (unsigned)s_offset[idx];  
  
  if (same_sm || (unsigned)target_sm == s_sm) {  
    // Своя бывшая SM — уже unclaim'нута в фазе 3.  
    pio_sm_claim(s_cfg.pio, (uint)target_sm);  
  } else {  
    pio_sm_claim(s_cfg.pio, (uint)target_sm);  
  }  
  s_sm         = (unsigned)target_sm;  
  s_sm_claimed = true;  
  return true;  
}  
  
// Удобный init: LO = freq точно, без IF-сдвига и NCO-смещения.  
// IF-сдвиг и зоны NCO нужны только приёмнику — задавайте их  
// через полный quad_vfo_init(&cfg).  
bool quad_vfo_init_simple(PIO pio, uint8_t pin_i) {  
  quad_vfo_cfg_t c;  
  quad_vfo_default_cfg(&c);  
  c.pio           = pio;  
  c.pin_i         = pin_i;  
  c.sm            = -1;  
  c.if_shift_hz   = 0;  
  c.nco_sweet_lo  = 0;  
  c.nco_sweet_hi  = 0;  
  c.nco_forbid_lo = 0;  
  c.nco_forbid_hi = 0;  
  c.nco_max_hz    = 0;   // 0 = режим точной частоты  
  return quad_vfo_init(&c);  
}  
  
// ---------------------------------------------------------------------------  
// Пересчёт делителя от реальной clk_sys и проверка NCO-ограничений.  
// Используется и на основном пути, и после отката PLL.  
// Возвращает false, если ни один делитель не удовлетворяет ограничениям.  
static bool recompute_div(uint64_t clk_sys_hz, uint32_t lo_hz,  
                          uint32_t *out_div_q8, int64_t *out_f_nco,  
                          bool *out_is_int) {  
  // Целый делитель  
  uint64_t D = (clk_sys_hz + 2ull * lo_hz) / (4ull * lo_hz);  
  if (D < 2)      D = 2;  
  if (D > 0xFFFF) D = 0xFFFF;  
  uint32_t div_int = (uint32_t)(D << 8);  
  
  // Дробный Q8  
  uint64_t q = (clk_sys_hz * 256ull + 2ull * lo_hz) / (4ull * lo_hz);  
  if (q < (2u << 8))      q = 2u << 8;  
  if (q > (0xFFFFu << 8)) q = 0xFFFFu << 8;  
  uint32_t div_frac = (uint32_t)q;  
  
  // Оцениваем оба варианта по NCO-ограничениям  
  uint32_t best_div = 0;  
  int64_t  best_nco = 0;  
  bool     best_int = true;  
  int64_t  best_abs = INT64_MAX;  
  
  for (int k = 0; k < 2; k++) {  
    uint32_t dq = (k == 0) ? div_int : div_frac;  
    uint64_t lo_got = (clk_sys_hz * 256ull) / (4ull * (uint64_t)dq);  
    int64_t  f_nco  = (int64_t)lo_hz - (int64_t)lo_got;  
    // В приёмнике f_nco считается от freq (freq = lo_hz + if_shift);  
    // в точном режиме lo_hz == freq — формула совпадает.  
    int64_t  f_tgt  = (s_cfg.nco_max_hz != 0)  
                      ? (int64_t)(lo_hz + s_cfg.if_shift_hz) - (int64_t)lo_got  
                      : f_nco;  
    int64_t  a      = (f_tgt < 0) ? -f_tgt : f_tgt;  
  
    bool nco_ok;  
    if (s_cfg.nco_max_hz == 0) {  
      nco_ok = (a <= (int64_t)QUAD_EXACT_MODE_TOL_HZ);  
    } else {  
      nco_ok = (f_tgt > 0 &&  
                f_tgt < (int64_t)s_cfg.nco_max_hz &&  
                !(f_tgt >= (int64_t)s_cfg.nco_forbid_lo &&  
                  f_tgt <= (int64_t)s_cfg.nco_forbid_hi));  
    }  
    if (!nco_ok) continue;  
    if (a < best_abs) { best_abs = a; best_div = dq; best_nco = f_tgt;  
                        best_int = (k == 0); }  
  }  
  
  if (best_div == 0) return false;  
  *out_div_q8 = best_div;  
  *out_f_nco  = best_nco;  
  *out_is_int = best_int;  
  return true;  
}  
  
// ---------------------------------------------------------------------------  
bool quad_vfo_start(uint32_t freq_hz) {  
  if (!s_have_cfg || !s_sm_claimed) return false;  
  if (freq_hz == 0)                 return false;  
  // Невалидный вход — ранний отказ БЕЗ остановки работающего генератора.  
  if (freq_hz <= s_cfg.if_shift_hz) return false;  
  
  // Целевая частота LO — ниже частоты приёма на величину ПЧ  
  // (в точном режиме if_shift_hz == 0 -> lo_hz == freq_hz).  
  uint32_t lo_hz = freq_hz - s_cfg.if_shift_hz;  
  Serial.printf("[QUAD] Целевая частота LO %lu\n", (unsigned long)lo_hz);  
  
  const uint64_t xosc_hz = s_cfg.xosc_hz;  
  // Эффективный потолок скана = min(паспорт, аппаратный барьер 420 МГц).  
  // Если паспорт не доказан (окно пустое) — потолок из pll_ceiling.  
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
  
  // Веса взвешенной суммы штрафов: W2 >> W3 >> W4 >> W5.  
  const uint64_t W2 = 1000000000ull;  // П2: удаление от сладкой зоны / |f_nco|  
  const uint64_t W3 = 1000000ull;     // П3: штраф за дробный делитель  
  const uint64_t W4 = 1000ull;        // П4: близость дроби к k/8  
  const uint64_t W5 = 1ull;           // П5: шина ниже максимума (за 1 МГц)  
  
  quad_vfo_pll_t best        = s_cfg.pll_nominal;  
  uint64_t  best_cost   = UINT64_MAX;  
  uint32_t  best_div_q8 = 0;  
  bool      best_is_int = true;  
  bool      best_legal  = false;  
  
  // Fallback «наименьшее зло»: лучший среди НЕлегальных.  
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
          if (vco < 750000000ULL || vco > vco_max) continue;  
          uint64_t clk = vco / pdiv;  
          if (clk < s_cfg.clk_sys_min_hz || clk > clk_max) continue;  
  
          // Два варианта делителя на эту шину: целый и дробный (Q8).  
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
            // В режиме приёмника ПЧ должна быть > 0 (LO строго ниже  
            // freq). В точном режиме допускаем любой знак ошибки —  
            // иначе отбрасываем само точное попадание.  
            if (s_cfg.nco_max_hz != 0 && lo_got >= freq_hz) continue;  
            int64_t f_nco = (int64_t)freq_hz - (int64_t)lo_got;  
  
            // nco_max_hz == 0 -> любой f_nco легален (режим точной  
            // частоты); иначе f_nco обязан быть в (0, nco_max).  
            if (s_cfg.nco_max_hz != 0 &&  
                (f_nco <= 0 || f_nco >= (int64_t)s_cfg.nco_max_hz))  
              continue;  
  
            uint32_t frac   = div_q8 & 0xFF;  
            bool     is_int = (frac == 0);  
  
            // --- П1: жёсткий запрет дырки [forbid_lo..forbid_hi] ---  
            // В точном режиме зоны нулевые — блок пропускаем,  
            // иначе f_nco==0 считалось бы "запретным".  
            if (s_cfg.nco_max_hz != 0 &&  
                f_nco >= (int64_t)s_cfg.nco_forbid_lo &&  
                f_nco <= (int64_t)s_cfg.nco_forbid_hi) {  
              uint64_t bad = (uint64_t)((int64_t)s_cfg.nco_forbid_hi - f_nco);  
              if (bad < fb_bad) {  
                fb_bad    = bad;  
                fb_best.refdiv = refdiv; fb_best.p1 = p1; fb_best.p2 = p2;  
                fb_best.fbdiv  = fbdiv;  fb_best.clk_sys_hz = (uint32_t)clk;  
                fb_best.vsel   = quad_vfo_vsel_for(clk);  
                fb_best.is_oc  = (clk > s_cfg.clk_sys_nominal_hz);  
                fb_div_q8 = div_q8;  
                fb_is_int = is_int;  
              }  
              continue;  
            }  
  
            // --- взвешенная сумма штрафов П2..П5 ---  
            uint64_t cost = 0;  
            if (s_cfg.nco_max_hz == 0) {  
              // Точная частота: главный критерий — минимум |f_nco|.  
              cost += (uint64_t)(f_nco < 0 ? -f_nco : f_nco) * W2;  
            } else {  
              if      (f_nco < (int64_t)s_cfg.nco_sweet_lo)  
                cost += (uint64_t)((int64_t)s_cfg.nco_sweet_lo - f_nco) * W2;  
              else if (f_nco > (int64_t)s_cfg.nco_sweet_hi)  
                cost += (uint64_t)(f_nco - (int64_t)s_cfg.nco_sweet_hi) * W2;  
            }  
            if (!is_int) cost += W3;                    // П3: дробность  
            if (!is_int) {                              // П4: k/8  
              int32_t m    = (int32_t)frac % 32;  
              int32_t dist = (m < 16) ? m : 32 - m;  
              cost += (uint64_t)(16 - dist) * W4;  
            }  
            cost += ((clk_max - clk) / 1000000ull) * W5; // П5  
  
            if (cost < best_cost) {  
              best_cost   = cost;  
              best_div_q8 = div_q8;  
              best_is_int = is_int;  
              best_legal  = true;  
              best.refdiv = refdiv; best.p1 = p1; best.p2 = p2;  
              best.fbdiv  = fbdiv;  best.clk_sys_hz = (uint32_t)clk;  
              best.vsel   = quad_vfo_vsel_for(clk);  
              best.is_oc  = (clk > s_cfg.clk_sys_nominal_hz);  
            }  
          }  
        }  
      }  
    }  
  }  
  
  // П6: пустой скан — не трогаем шину и честно отказываемся,  
  // работающий генератор (если был) сохраняется.  
  if (!best_legal && fb_bad == UINT64_MAX) {  
    Serial.println("[QUAD] ОШИБКА: скан не дал ни одного кандидата — "  
                   "проверьте clk_sys_max_hz/pll_ceiling. LO не тронут.");  
    return false;  
  }  
  if (!best_legal) {  
    Serial.println("[QUAD] ВНИМАНИЕ: нет кандидата с дыркой вне "  
                   "запретной зоны — ставлю наименьшее зло");  
    best        = fb_best;  
    best_div_q8 = fb_div_q8;  
    best_is_int = fb_is_int;  
  }  
  
  // === Точка невозврата: дальше отказ -> fail_path ===  
  pio_sm_set_enabled(s_cfg.pio, s_sm, false);  
  
  uint32_t vsel = best.vsel;  
  if (vsel == 0 || vsel < (uint32_t)VREG_VOLTAGE_1_10)  
    vsel = quad_vfo_vsel_for(best.clk_sys_hz);  
  
  bool pll_ok = quad_vfo_set_clk_sys(&best, vsel,  
                                   &s_cfg.pll_nominal, xosc_hz);  
  
  // Делитель пересчитываем от РЕАЛЬНОЙ шины — и на успехе, и после  
  // отката на номинал (там реальная шина другая).  
  uint64_t clk_sys_hz = clock_get_hz(clk_sys);  
  uint32_t div_q8;  
  int64_t  f_nco;  
  bool     is_int;  
  
  if (!recompute_div(clk_sys_hz, lo_hz, &div_q8, &f_nco, &is_int)) {  
    Serial.println("[QUAD] ОШИБКА: после перепрошивки шины ни один "  
                   "делитель не удовлетворяет NCO-ограничениям");  
    fail_path();  
    return false;  
  }  
  
  if (!pll_ok) {  
    // Шина на номинале (или XOSC при неудачном отката) — делитель  
    // пересчитан и ограничения перепроверены, запускаем компромиссный LO.  
    Serial.println("[QUAD] ВНИМАНИЕ: целевая шина не установлена — "  
                   "LO пересчитан под реальный clk_sys");  
  }  
  
  s_lo_actual_hz = (uint32_t)((clk_sys_hz * 256ull) /  
                              (4ull * (uint64_t)div_q8));  
  {  
    // Ошибка в миллигерцах: целевое LO*1000 минус фактическое.  
    uint64_t lo_mhz = (clk_sys_hz * 256ull * 1000ull) /  
                      (4ull * (uint64_t)div_q8);  
    s_lo_error_mhz = (int32_t)((int64_t)lo_hz * 1000 - (int64_t)lo_mhz);  
  }  
  
  {  
    uint32_t frac = div_q8 & 0xFF;  
    bool sweet = (s_cfg.nco_max_hz != 0 &&  
                  f_nco >= (int64_t)s_cfg.nco_sweet_lo &&  
                  f_nco <= (int64_t)s_cfg.nco_sweet_hi);  
    Serial.printf("[QUAD] div=%lu+%lu/256 (%s)  lo=%lu Гц  "  
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