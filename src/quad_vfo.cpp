// quad_vfo.cpp  
// ============================================================================  
// Автономный квадратурный LO. Полный перенос логики soft_rx.cpp:  
// сканер PLL (критерии П1..П6), fallback "наименьшее зло", пересчёт  
// clkdiv от реальной clock_get_hz, направление пинов — последним.  
// ============================================================================  
#include "quad_vfo.h"  
#include "hardware/clocks.h"  
#include "hardware/vreg.h"  
  
// ---- PIO-программа: 4 такта на период, 2 соседних пина ----------------------  
static const uint16_t pio_quad_instructions[] = {  
  0xe001,  // set pins, 0b01: Q=1 I=0  
  0xe003,  // set pins, 0b11: Q=1 I=1  
  0xe002,  // set pins, 0b10: Q=0 I=1  
  0xe000,  // set pins, 0b00: Q=0 I=0  
};  
static const pio_program_t pio_quad_program = {  
  .instructions = pio_quad_instructions,  
  .length       = 4,  
  .origin       = -1,  
  .pio_version  = 0  
};  
  
// ---- Состояние -------------------------------------------------------------  
static quad_vfo_cfg_t s_cfg;  
static bool     s_have_cfg        = false;  
static unsigned s_sm              = 0;  
static unsigned s_offset          = 0;  
static bool     s_program_loaded  = false;  
static bool     s_active          = false;  
static uint32_t s_lo_actual_hz    = 0;  
  
// ---------------------------------------------------------------------------  
void quad_vfo_default_cfg(quad_vfo_cfg_t *cfg) {  
  memset(cfg, 0, sizeof(*cfg));  
  cfg->pio   = pio1;  
  cfg->sm    = -1;  
  cfg->pin_i = 0;                       // I=GP0, Q=GP1  
  
  // Профили из soft_tx.cpp: nominal 133M, ceiling 380M@1.30V.  
  cfg->pll_nominal = { 133, 6, 2, 133000000u,  
                       (uint32_t)VREG_VOLTAGE_DEFAULT, false, 1 };  
  cfg->pll_ceiling = {  95, 3, 1, 380011083u,  
                       (uint32_t)VREG_VOLTAGE_1_30,    true,  1 };  
  
  cfg->xosc_hz            = 12000000ULL;  
  cfg->clk_sys_min_hz     = 240000000ULL;  
  cfg->clk_sys_max_hz     = 133000000ULL;   // "не доказан" → ceiling  
  cfg->pll_vco_max_hz     = 1600000000ULL;  
  cfg->clk_sys_nominal_hz = 133000000ULL;  
  
  cfg->if_shift_hz   = 11000u;              // было RX_IF_SHIFT_HZ  
  cfg->nco_max_hz    = 62500u;  
  cfg->nco_sweet_lo  = 9600u;  
  cfg->nco_sweet_hi  = 14500u;  
  cfg->nco_forbid_lo = 0u;  
  cfg->nco_forbid_hi = 5000u;  
}  
  
// ---------------------------------------------------------------------------  
bool quad_vfo_init(const quad_vfo_cfg_t *cfg) {  
  if (!cfg) return false;  
  if (cfg->pin_i + 1 > 29) {              // пины должны быть смежными  
    Serial.println("[QUAD] ОШИБКА: pin_i+1 > 29 — нужны смежные пины");  
    return false;  
  }  
  if (cfg->sm < -1 || cfg->sm > 3) {  
    Serial.println("[QUAD] ОШИБКА: sm вне {-1,0..3}");  
    return false;  
  }  
  if (cfg->sm >= 0 && pio_sm_is_claimed(cfg->pio, (uint)cfg->sm)) {  
    Serial.printf("[QUAD] ОШИБКА: pio%d SM%d уже занята\n",  
                  pio_get_index(cfg->pio), cfg->sm);  
    return false;  
  }  
  
  s_cfg      = *cfg;  
  s_have_cfg = true;  
  
  if (!s_program_loaded) {  
    s_offset = pio_add_program(s_cfg.pio, &pio_quad_program);  
    s_program_loaded = true;  
  }  
  if (s_cfg.sm < 0) {  
    s_sm = pio_claim_unused_sm(s_cfg.pio, true);  
  } else {  
    pio_sm_claim(s_cfg.pio, (uint)s_cfg.sm);  
    s_sm = (unsigned)s_cfg.sm;  
  }  
  // Пины в output НЕ переводим — направление ставится последним в start.  
  return true;  
}  
  
bool quad_vfo_init_simple(PIO pio, uint8_t pin_i) {  
  quad_vfo_cfg_t cfg;  
  quad_vfo_default_cfg(&cfg);  
  cfg.pio   = pio;  
  cfg.pin_i = pin_i;  
  cfg.sm    = -1;  
  return quad_vfo_init(&cfg);  
}  
  
// ---------------------------------------------------------------------------  
// ЗАПУСК: скан PLL → делитель → конфигурация SM (критерии П1..П6)  
// ---------------------------------------------------------------------------  
bool quad_vfo_start(uint32_t freq_hz) {  
  if (!s_have_cfg) return false;  
  
  uint32_t lo_hz = (freq_hz > s_cfg.if_shift_hz)  
                   ? (freq_hz - s_cfg.if_shift_hz) : freq_hz;  
  Serial.printf("[QUAD] Целевая частота LO %lu\n", (unsigned long)lo_hz);  
  
  const uint64_t xosc_hz = s_cfg.xosc_hz;  
  
  // Потолок скана = min(паспорт, 420M барьер); пустое окно → pll_ceiling.  
  uint64_t clk_max = (s_cfg.clk_sys_max_hz < 420000000ULL)  
                     ? s_cfg.clk_sys_max_hz : 420000000ULL;  
  if (clk_max < s_cfg.clk_sys_min_hz) {  
    clk_max = s_cfg.pll_ceiling.clk_sys_hz;  
    Serial.printf("[QUAD] паспорт clk_max=%lu МГц не доказан — "  
                  "потолок из pll_ceiling=%lu МГц\n",  
                  (unsigned long)(s_cfg.clk_sys_max_hz / 1000000UL),  
                  (unsigned long)(clk_max / 1000000ULL));  
  }  
  const uint64_t vco_max = (s_cfg.pll_vco_max_hz < 4300000000ULL)  
                           ? s_cfg.pll_vco_max_hz : 4300000000ULL;  
  
  // Веса лексикографики: W2 >> W3 >> W4 >> W5.  
  const uint64_t W2 = 1000000000ull;  // удаление от сладкой зоны (за 1 Гц)  
  const uint64_t W3 = 1000000ull;     // штраф за дробный делитель  
  const uint64_t W4 = 1000ull;        // близость дроби к k/8  
  const uint64_t W5 = 1ull;           // шина ниже максимума (за 1 МГц)  
  
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
          if (vco < 750000000ULL || vco > vco_max) continue;  
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
            if (lo_got >= freq_hz) continue;            // ПЧ должна быть > 0  
            int64_t f_nco = (int64_t)freq_hz - (int64_t)lo_got;  
            if (f_nco <= 0 || f_nco >= (int64_t)s_cfg.nco_max_hz) continue;  
  
            uint32_t frac   = div_q8 & 0xFFu;  
            bool     is_int = (frac == 0);  
  
            // П1: жёсткий запрет дырки в звуке [forbid_lo..forbid_hi]  
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
  
            // стоимость легального кандидата  
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
  
  // П6: пустой скан — НЕ прошиваем, LO не тронут.  
  if (!best_legal && fb_bad == UINT64_MAX) {  
    Serial.println("[QUAD] ОШИБКА: скан не дал ни одного кандидата — "  
                   "проверьте clk_sys_max_hz/pll_ceiling. LO не тронут.");  
    s_lo_actual_hz = 0;  
    return false;  
  }  
  if (!best_legal) {  
    Serial.println("[QUAD] ВНИМАНИЕ: нет кандидата с дыркой вне "  
                   "запретной зоны — ставлю наименьшее зло");  
    best        = fb_best;  
    best_div_q8 = fb_div_q8;  
    best_is_int = fb_is_int;  
  }  
  
  pio_sm_set_enabled(s_cfg.pio, s_sm, false);  
  
  uint32_t vsel = best.vsel;  
  if (vsel == 0 || vsel < (uint32_t)VREG_VOLTAGE_1_10)  
    vsel = quad_vfo_vsel_for(best.clk_sys_hz);  
  quad_vfo_set_clk_sys(&best, vsel, &s_cfg.pll_nominal, xosc_hz);  
  
  // Делитель от РЕАЛЬНОЙ шины.  
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
  s_lo_actual_hz = (uint32_t)((clk_sys_hz * 256ull) /  
                              (4ull * (uint64_t)div_q8));  
  
  {  
    int32_t  f_nco  = (int32_t)freq_hz - (int32_t)s_lo_actual_hz;  
    uint32_t frac   = div_q8 & 0xFFu;  
    bool     sweet  = (f_nco >= (int32_t)s_cfg.nco_sweet_lo &&  
                       f_nco <= (int32_t)s_cfg.nco_sweet_hi);  
    Serial.printf("[QUAD] clk_sys=%lu, div=%lu+%lu/256 (%s), lo_actual=%lu, "  
                  "f_nco=%ld, sweet=%d\n",  
                  (unsigned long)clk_sys_hz,  
                  (unsigned long)(div_q8 >> 8), (unsigned long)frac,  
                  frac ? "дробный" : "целый",  
                  (unsigned long)s_lo_actual_hz,  
                  (long)f_nco, sweet ? 1 : 0);  
    if (f_nco > (int32_t)s_cfg.nco_max_hz ||  
        f_nco < -(int32_t)s_cfg.nco_max_hz) {  
      Serial.printf("[QUAD] ВНИМАНИЕ: |f_nco| > nco_max — приём невозможен\n");  
    }  
  }  
  
  // === Конфигурация SM под квадратуру ===  
  s_active = false;  
  
  pio_sm_config cq = pio_get_default_sm_config();  
  sm_config_set_wrap(&cq, s_offset + 0, s_offset + 3);  
  sm_config_set_set_pins(&cq, s_cfg.pin_i, 2);   // I=base, Q=base+1  
  sm_config_set_clkdiv(&cq, (float)div_q8 / 256.0f);  
  pio_gpio_init(s_cfg.pio, s_cfg.pin_i);  
  pio_gpio_init(s_cfg.pio, s_cfg.pin_i + 1);  
  
  pio_sm_init(s_cfg.pio, s_sm, s_offset, &cq);  
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
  s_active = false;  
}  
  
uint32_t quad_vfo_actual_hz(void) { return s_lo_actual_hz; }  
bool     quad_vfo_active(void)    { return s_active; }  
  
// ---------------------------------------------------------------------------  
void quad_vfo_debug(void) {  
  if (!s_have_cfg) { Serial.println("[LODBG] не инициализирован"); return; }  
  uint32_t meas_clk = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS) * 1000u;  
  uint32_t cdiv     = s_cfg.pio->sm[s_sm].clkdiv;  
  
  Serial.printf("[LODBG] clk_sys: cached=%lu kHz  measured=%lu kHz\n",  
                (unsigned long)(clock_get_hz(clk_sys) / 1000ULL),  
                (unsigned long)(meas_clk / 1000u));  
  Serial.printf("[LODBG] PIO=%d SM=%d  clkdiv=0x%08lX (int=%lu frac=%lu)\n",  
                pio_get_index(s_cfg.pio), (int)s_sm,  
                (unsigned long)cdiv,  
                (unsigned long)(cdiv >> 16),  
                (unsigned long)((cdiv >> 8) & 0xFF));  
  Serial.printf("[LODBG] SM ctrl=0x%08lX enabled=%d  execctrl=0x%08lX\n",  
                (unsigned long)s_cfg.pio->ctrl,  
                (int)((s_cfg.pio->ctrl & (1u << s_sm)) != 0),  
                (unsigned long)s_cfg.pio->sm[s_sm].execctrl);  
  Serial.printf("[LODBG] lo_hz=%lu  active=%d\n",  
                (unsigned long)s_lo_actual_hz, (int)s_active);  
}
