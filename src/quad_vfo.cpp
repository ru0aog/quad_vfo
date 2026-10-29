// quad_vfo.cpp  
// ============================================================================  
// Квадратурный LO: сканер PLL + PIO. Порт src/vfo/soft_rx.cpp (RU0AOG).  
// Правки аудита: транзакционный init (валидация и предвыбор SM ДО  
// освобождения старых ресурсов), единый fail_path, проверка результата  
// set_clk_sys + повторная проверка NCO, VCO-cap 1600 МГц.  
// ============================================================================  
#include "quad_vfo.h"  
#include "hardware/clocks.h"  
#include "hardware/vreg.h"  
  
// ---------------------------------------------------------------------------  
// 4-тактовая PIO-программа: set pins (bit0 -> I, bit1 -> Q)  
//   01: I=1 Q=0   11: I=1 Q=1   10: I=0 Q=1   00: I=0 Q=0  
//   I: 1,1,0,0  Q: 0,1,1,0 — Q отстаёт от I на 90°.  
// ---------------------------------------------------------------------------  
static const uint16_t pio_quad_instructions[4] = {  
  (uint16_t)pio_encode_set(pio_pins, 0b01),  
  (uint16_t)pio_encode_set(pio_pins, 0b11),  
  (uint16_t)pio_encode_set(pio_pins, 0b10),  
  (uint16_t)pio_encode_set(pio_pins, 0b00),  
};  
static const struct pio_program pio_quad_program = {  
  .instructions = pio_quad_instructions,  
  .length = 4,  
  .origin = -1,  
};  
  
// Аппаратный предел VCO pll_sys по datasheet RP2040.  
#define QUAD_VCO_HW_MAX_HZ  1600000000ULL  
#define QUAD_VCO_HW_MIN_HZ   750000000ULL   // рабочее окно исходника  
  
// ---------------------------------------------------------------------------  
static quad_vfo_cfg_t s_cfg;  
static bool     s_have_cfg    = false;  
static unsigned s_sm          = 0;  
static bool     s_sm_claimed  = false;  
static bool     s_active      = false;  
static uint32_t s_lo_actual_hz = 0;  
static int32_t  s_lo_error_mhz = 0;  
  
static int      s_offset[2]   = { -1, -1 };   // по pio_get_index()  
static int      s_cur_offset  = -1;  
  
// ---------------------------------------------------------------------------  
void quad_vfo_default_cfg(quad_vfo_cfg_t *cfg) {  
  cfg->pio   = pio1;  
  cfg->sm    = -1;  
  cfg->pin_i = 0;  
  
  // NB: сверьте pll_ceiling с инициализатором в вашем soft_tx.cpp.  
  cfg->pll_nominal = { 133, 6, 2, 133000000u,  
                       (uint32_t)VREG_VOLTAGE_DEFAULT, false, 1 };  
  cfg->pll_ceiling = { 95, 3, 1, 380000000u,  
                       (uint32_t)VREG_VOLTAGE_1_30, true, 1 };  
  
  cfg->xosc_hz            = 12000000ULL;  
  cfg->clk_sys_min_hz     = 240000000ULL;  
  cfg->clk_sys_max_hz     = 133000000ULL;   // "не доказан" -> pll_ceiling  
  cfg->pll_vco_max_hz     = 1600000000ULL;  
  cfg->clk_sys_nominal_hz = 133000000ULL;  
  
  cfg->if_shift_hz   = 11000u;  
  cfg->nco_max_hz    = 62500u;  
  cfg->nco_sweet_lo  = 9600u;  
  cfg->nco_sweet_hi  = 14500u;  
  cfg->nco_forbid_lo = 0u;  
  cfg->nco_forbid_hi = 5000u;  
}  
  
// ---------------------------------------------------------------------------  
// Валидация PLL-профиля: диапазоны + согласованность clk_sys_hz с  
// fbdiv/refdiv/p1/p2/xosc (расхождение <= 1% — округление pll_init).  
// ---------------------------------------------------------------------------  
static bool pll_profile_ok(const quad_vfo_pll_t *p) {  
  if (!p) return false;  
  if (p->refdiv == 0)            return false;  
  if (p->p1 < 2 || p->p1 > 7)    return false;  
  if (p->p2 < 1 || p->p2 > 7)    return false;  
  if (p->fbdiv < 16 || p->fbdiv > 320) return false;  
  return true;  
}  
  
static bool pll_profile_consistent(const quad_vfo_pll_t *p, uint64_t xosc) {  
  uint64_t calc = ((uint64_t)p->fbdiv * xosc)  
                  / ((uint64_t)p->refdiv * p->p1 * p->p2);  
  uint64_t diff = (calc > p->clk_sys_hz) ? (calc - p->clk_sys_hz)  
                                         : (p->clk_sys_hz - calc);  
  return diff * 100 <= p->clk_sys_hz;    // <= 1%  
}  
  
static bool pio_ok(PIO p) { return p == pio0 || p == pio1; }  
  
// ---------------------------------------------------------------------------  
// Единый путь отказа: генератор гарантированно остановлен, состояние  
// согласовано (active=false, actual=0). Применять только после «точки  
// невозврата» — ранние отказы до старта не трогают работающий LO.  
// ---------------------------------------------------------------------------  
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
// Транзакционный init:  
//   фаза 1 — валидация нового cfg;  
//   фаза 2 — проверка/предвыбор SM (ДО освобождения старых ресурсов);  
//   фаза 3 — освобождение старой конфигурации;  
//   фаза 4 — захват новых ресурсов.  
// При false до фазы 3 прежнее состояние сохранено.  
// ---------------------------------------------------------------------------  
bool quad_vfo_init(const quad_vfo_cfg_t *cfg) {  
  // ---- фаза 1: валидация ----  
  if (!cfg)                                  return false;  
  if (!pio_ok(cfg->pio))                     return false;  
  if (cfg->pin_i + 1 > 29)                   return false;  
  if (cfg->sm < -1 || cfg->sm > 3)           return false;  
  if (cfg->xosc_hz == 0)                     return false;  
  if (cfg->nco_max_hz == 0)                  return false;  
  if (cfg->nco_sweet_lo > cfg->nco_sweet_hi) return false;  
  if (cfg->nco_forbid_lo > cfg->nco_forbid_hi) return false;  
  if (cfg->if_shift_hz >= cfg->nco_max_hz)   return false;  
  if (cfg->clk_sys_min_hz == 0)              return false;  
  if (cfg->pll_vco_max_hz == 0)              return false;  
  if (!pll_profile_ok(&cfg->pll_nominal) ||  
      !pll_profile_consistent(&cfg->pll_nominal, cfg->xosc_hz))  
    return false;  
  if (!pll_profile_ok(&cfg->pll_ceiling) ||  
      !pll_profile_consistent(&cfg->pll_ceiling, cfg->xosc_hz))  
    return false;  
  
  // ---- фаза 2: предвыбор SM ----  
  // own_sm_free: старая SM на этом же PIO будет освобождена в фазе 3 —  
  // её можно считать доступной кандидаткой.  
  bool own_sm_free = (s_sm_claimed && s_cfg.pio == cfg->pio);  
  int  target_sm   = -1;  
  
  if (cfg->sm >= 0) {  
    bool free_sm = !pio_sm_is_claimed(cfg->pio, (uint)cfg->sm);  
    bool own_sm  = (own_sm_free && (unsigned)cfg->sm == s_sm);  
    if (!free_sm && !own_sm) return false;   // занята чужим кодом  
    target_sm = cfg->sm;  
  } else {  
    for (int sm = 0; sm < 4; sm++) {  
      if (!pio_sm_is_claimed(cfg->pio, (uint)sm) ||  
          (own_sm_free && (unsigned)sm == s_sm)) {  
        target_sm = sm;  
        break;  
      }  
    }  
    if (target_sm < 0) return false;   // свободных SM нет — старое не тронуто  
  }  
  
  // ---- фаза 3: освобождение старой конфигурации ----  
  if (s_sm_claimed) {  
    quad_vfo_stop();  
    pio_sm_unclaim(s_cfg.pio, s_sm);  
    s_sm_claimed = false;  
  }  
  
  s_cfg      = *cfg;  
  s_have_cfg = true;  
  s_sm       = (unsigned)target_sm;  
  pio_sm_claim(s_cfg.pio, s_sm);  
  s_sm_claimed = true;  
  
  // Программа грузится в каждый PIO-блок отдельно; offset кэшируется  
  // per-PIO и переиспользуется между init.  
  int idx = (int)pio_get_index(s_cfg.pio);  
  if (s_offset[idx] < 0)  
    s_offset[idx] = pio_add_program(s_cfg.pio, &pio_quad_program);  
  s_cur_offset = s_offset[idx];  
  
  return true;  
}  
  
bool quad_vfo_init_simple(PIO pio, uint8_t pin_i) {  
  quad_vfo_cfg_t cfg;  
  quad_vfo_default_cfg(&cfg);  
  cfg.pio   = pio;  
  cfg.pin_i = pin_i;  
  return quad_vfo_init(&cfg);  
}  
  
// ---------------------------------------------------------------------------  
// Пересчёт делителя от реальной шины + проверка NCO-зон.  
// ---------------------------------------------------------------------------  
static bool recompute_div(uint64_t clk_sys_hz, uint32_t lo_hz,  
                          uint32_t freq_hz, bool prefer_int,  
                          uint32_t *div_q8_out) {  
  uint32_t div_q8;  
  if (prefer_int) {  
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
  *div_q8_out = div_q8;  
  
  // LO в миллигерцах — не теряем дробную часть на промежуточных этапах.  
  uint64_t lo_mhz = (clk_sys_hz * 256ull * 1000ull) /  
                    (4ull * (uint64_t)div_q8);  
  s_lo_actual_hz = (uint32_t)(lo_mhz / 1000ull);  
  s_lo_error_mhz = (int32_t)((int64_t)lo_hz * 1000ll - (int64_t)lo_mhz);  
  
  int64_t f_nco = (int64_t)freq_hz - (int64_t)s_lo_actual_hz;  
  if (f_nco <= 0 || f_nco >= (int64_t)s_cfg.nco_max_hz) return false;  
  if (f_nco >= (int64_t)s_cfg.nco_forbid_lo &&  
      f_nco <= (int64_t)s_cfg.nco_forbid_hi) return false;  
  return true;  
}  
  
// ---------------------------------------------------------------------------  
bool quad_vfo_start(uint32_t freq_hz) {  
  if (!s_have_cfg) return false;  
  
  // Ранний отказ: недопустимая частота — работающий генератор НЕ трогаем.  
  if (freq_hz == 0 || freq_hz <= s_cfg.if_shift_hz) {  
    Serial.printf("[QUAD] ОШИБКА: freq=%lu <= if_shift=%lu — "  
                  "отклонено\n", (unsigned long)freq_hz,  
                  (unsigned long)s_cfg.if_shift_hz);  
    return false;  
  }  
  uint32_t lo_hz = freq_hz - s_cfg.if_shift_hz;  
  Serial.printf("[QUAD] Целевая частота LO %lu\n", (unsigned long)lo_hz);  
  
  const uint64_t xosc_hz = s_cfg.xosc_hz;  
  
  uint64_t clk_max = (s_cfg.clk_sys_max_hz < 420000000ULL)  
                     ? s_cfg.clk_sys_max_hz : 420000000ULL;  
  if (clk_max < s_cfg.clk_sys_min_hz) {  
    clk_max = s_cfg.pll_ceiling.clk_sys_hz;  
    Serial.printf("[QUAD] паспорт clk_max=%lu МГц не доказан — "  
                  "потолок из pll_ceiling=%lu МГц\n",  
                  (unsigned long)(s_cfg.clk_sys_max_hz / 1000000UL),  
                  (unsigned long)(clk_max / 1000000ULL));  
  }  
  // Аппаратный барьер VCO — datasheet RP2040: 1600 МГц.  
  const uint64_t vco_max = (s_cfg.pll_vco_max_hz < QUAD_VCO_HW_MAX_HZ)  
                           ? s_cfg.pll_vco_max_hz : QUAD_VCO_HW_MAX_HZ;  
  
  // Взвешенная сумма штрафов (W2 >> W3 >> W4 >> W5).  
  const uint64_t W2 = 1000000000ull;  
  const uint64_t W3 = 1000000ull;  
  const uint64_t W4 = 1000ull;  
  const uint64_t W5 = 1ull;  
  
  quad_vfo_pll_t best        = s_cfg.pll_nominal;  
  uint64_t       best_cost   = UINT64_MAX;  
  uint32_t       best_div_q8 = 0;  
  bool           best_is_int = true;  
  bool           best_legal  = false;  
  
  quad_vfo_pll_t fb_best   = s_cfg.pll_nominal;  
  uint64_t       fb_bad    = UINT64_MAX;  
  uint32_t       fb_div_q8 = 0;  
  bool           fb_is_int = true;  
  
  for (uint32_t refdiv = 1; refdiv <= 3; refdiv++) {  
    uint64_t ref_hz = xosc_hz / refdiv;  
    for (uint32_t p1 = 2; p1 <= 7; p1++) {  
      for (uint32_t p2 = 1; p2 <= 7; p2++) {  
        if (p1 < p2) continue;  
        uint32_t pdiv = p1 * p2;  
        for (uint32_t fbdiv = 16; fbdiv <= 320; fbdiv++) {  
          uint64_t vco = ref_hz * fbdiv;  
          if (vco < QUAD_VCO_HW_MIN_HZ || vco > vco_max) continue;  
          uint64_t clk = vco / pdiv;  
          if (clk < s_cfg.clk_sys_min_hz || clk > clk_max) continue;  
  
          for (int frac_mode = 0; frac_mode < 2; frac_mode++) {  
            uint32_t div_q8;  
            if (frac_mode == 0) {  
              uint64_t D = (clk + 2ull * lo_hz) / (4ull * lo_hz);  
              if (D < 2)      D = 2;  
              if (D > 0xFFFF) D = 0xFFFF;  
              div_q8 = (uint32_t)(D << 8);  
            } else {  
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
  
  if (!best_legal && fb_bad == UINT64_MAX) {  
    // Ранний отказ: LO не тронут, работающий генератор сохраняется.  
    Serial.println("[QUAD] ОШИБКА: скан не дал ни одного кандидата — "  
                   "LO не тронут.");  
    return false;  
  }  
  if (!best_legal) {  
    Serial.println("[QUAD] ВНИМАНИЕ: нет кандидата вне запретной зоны — "  
                   "ставлю наименьшее зло (дырка в полосе!)");  
    best        = fb_best;  
    best_div_q8 = fb_div_q8;  
    best_is_int = fb_is_int;  
  }  
  (void)best_div_q8;  
  
  // === Точка невозврата: дальше любой отказ -> fail_path() ===  
  pio_sm_set_enabled(s_cfg.pio, s_sm, false);  
  s_active = false;  
  
  uint32_t vsel = best.vsel;  
  if (vsel == 0 || vsel < (uint32_t)VREG_VOLTAGE_1_10)  
    vsel = quad_vfo_vsel_for(best.clk_sys_hz);  
  
  bool pll_ok = quad_vfo_set_clk_sys(&best, vsel,  
                                     &s_cfg.pll_nominal, xosc_hz);  
  
  uint64_t clk_sys_hz = clock_get_hz(clk_sys);  
  uint32_t div_q8;  
  bool nco_ok = recompute_div(clk_sys_hz, lo_hz, freq_hz,  
                              best_is_int, &div_q8);  
  
  if (!pll_ok) {  
    Serial.println("[QUAD] ВНИМАНИЕ: clk_sys не подтверждён — "  
                   "откат на номинал/XOSC, перепроверяю NCO");  
    if (!nco_ok) {  
      Serial.println("[QUAD] ОШИБКА: на восстановленной шине NCO вне "  
                     "допустимых зон — LO не запущен");  
      fail_path();  
      return false;  
    }  
  } else if (!nco_ok) {  
    Serial.println("[QUAD] ОШИБКА: после пересчёта делителя NCO вне "  
                   "допустимых зон — LO не запущен");  
    fail_path();  
    return false;  
  }  
  
  {  
    int32_t  f_nco  = (int32_t)freq_hz - (int32_t)s_lo_actual_hz;  
    uint32_t frac   = div_q8 & 0xFFu;  
    bool     sweet  = (f_nco >= (int32_t)s_cfg.nco_sweet_lo &&  
                       f_nco <= (int32_t)s_cfg.nco_sweet_hi);  
    Serial.printf("[QUAD] clk_sys=%lu, div=%lu+%lu/256 (%s), lo_actual=%lu, "  
                  "err=%ld мГц, f_nco=%ld, sweet=%d\n",  
                  (unsigned long)clk_sys_hz,  
                  (unsigned long)(div_q8 >> 8), (unsigned long)frac,  
                  frac ? "дробный" : "целый",  
                  (unsigned long)s_lo_actual_hz,  
                  (long)s_lo_error_mhz,  
                  (long)f_nco, sweet ? 1 : 0);  
    if (f_nco > (int32_t)s_cfg.nco_max_hz ||  
        f_nco < -(int32_t)s_cfg.nco_max_hz) {  
      Serial.println("[QUAD] ВНИМАНИЕ: |f_nco| > nco_max — "  
                     "приём невозможен");  
    }  
  }  
  
  // === Конфигурация SM под квадратуру ===  
  pio_sm_config cq = pio_get_default_sm_config();  
  sm_config_set_wrap(&cq, s_cur_offset + 0, s_cur_offset + 3);  
  sm_config_set_set_pins(&cq, s_cfg.pin_i, 2);   // I=base, Q=base+1  
  sm_config_set_clkdiv(&cq, (float)div_q8 / 256.0f);  
  pio_gpio_init(s_cfg.pio, s_cfg.pin_i);  
  pio_gpio_init(s_cfg.pio, s_cfg.pin_i + 1);  
  
  pio_sm_init(s_cfg.pio, s_sm, s_cur_offset, &cq);  
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