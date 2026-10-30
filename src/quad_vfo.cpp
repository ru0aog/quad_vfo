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
// выключены, сканер выбирает делитель по минимуму |f_nco|, допуск при
// пересчёте — половина шага между соседними Q8-делителями.
//
// Жизненный цикл: offset программы хранится per-PIO; повторный init
// транзакционен — валидация, проверка доступности SM и памяти PIO
// выполняются ДО освобождения прежних ресурсов; новая SM захватывается
// до остановки старой. Отказы после "точки невозврата" приводят
// состояние в согласованный вид через fail_path().
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
  
// Электрический предел GPIO: выше ~45–50 МГц slew-rate пинов  
// превращает квадратурный меандр в ослабленный синус с разваленной  
// фазой — генерация смысла не имеет.  
#define QUAD_VFO_LO_MAX_HZ 45000000u

// ---------------------------------------------------------------------------
// PIO-программа квадратурного LO: 4 инструкции, set pins n.
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
static int32_t  s_lo_error_hz = 0;
  
// Фактически применённый профиль PLL из последнего успешного старта
static uint16_t s_pll_refdiv = 0;  
static uint16_t s_pll_fbdiv  = 0;  
static uint8_t  s_pll_p1     = 0;  
static uint8_t  s_pll_p2     = 0;  
static bool     s_pll_nominal_used = false;

static int      s_offset[2]    = { -1, -1 };
static unsigned s_cur_offset   = 0;

// Тихий режим: подавляет весь Serial-вывод библиотеки ([QUAD]) и,  
// через quad_vfo_pll_set_quiet(), вывод PLL-подсистемы ([PLL]).  
// Нужен при перестройке энкодером — чтобы не спамить терминал.  
static bool s_quiet = false;  
#define QV_LOG(...)  do { if (!s_quiet) Serial.printf(__VA_ARGS__); } while (0)

// ---------------------------------------------------------------------------
static bool pio_ok(PIO p) { return p == pio0 || p == pio1; }

static bool pll_profile_ok(const quad_vfo_pll_t *p, uint64_t xosc_hz) {
  if (p->refdiv < 1 || p->refdiv > 63) return false;
  if (p->p1 < 2 || p->p1 > 7) return false;
  if (p->p2 < 1 || p->p2 > 7) return false;
  if (p->fbdiv < 16 || p->fbdiv > 320) return false;
  if (p->clk_sys_hz == 0) return false;
  uint64_t vco = (xosc_hz / p->refdiv) * p->fbdiv;
  if (vco < QUAD_VCO_HW_MIN_HZ || vco > QUAD_VCO_HW_MAX_HZ) return false;
  return true;
}

static bool pll_profile_consistent(const quad_vfo_pll_t *p,
                                   uint64_t xosc_hz) {
  uint64_t denom = (uint64_t)p->refdiv * p->p1 * p->p2;
  if (denom == 0) return false;
  uint64_t calc = ((uint64_t)p->fbdiv * xosc_hz) / denom;
  uint64_t diff = (calc > p->clk_sys_hz) ? (calc - p->clk_sys_hz)
                                         : (p->clk_sys_hz - calc);
  return diff <= p->clk_sys_hz / 100;
}

// ---------------------------------------------------------------------------
static void fail_path(void) {
  pio_sm_set_enabled(s_cfg.pio, s_sm, false);
  pio_sm_set_consecutive_pindirs(s_cfg.pio, s_sm, s_cfg.pin_i, 2, false);
  gpio_set_dir(s_cfg.pin_i, GPIO_IN);
  gpio_set_dir(s_cfg.pin_i + 1, GPIO_IN);
  s_active = false;
  s_lo_actual_hz = 0;
  s_lo_error_hz = 0;
  s_pll_refdiv = s_pll_fbdiv = 0;  
  s_pll_p1 = s_pll_p2 = 0;  
  s_pll_nominal_used = false;
}

// ---------------------------------------------------------------------------
void quad_vfo_default_cfg(quad_vfo_cfg_t *cfg) {
  if (!cfg) return;
  memset(cfg, 0, sizeof(*cfg));

  cfg->pio = pio1;
  cfg->sm = -1;
  cfg->pin_i = 0;

  cfg->pll_nominal = { 133, 6, 2, 133000000u,
                       (uint32_t)VREG_VOLTAGE_DEFAULT, false, 1 };
  cfg->pll_ceiling = { 95, 3, 1, 380000000u,
                       (uint32_t)VREG_VOLTAGE_1_30, true, 1 };

  cfg->xosc_hz = 12000000ULL;
  cfg->clk_sys_min_hz = 240000000ULL;
  cfg->clk_sys_max_hz = 133000000ULL;
  cfg->pll_vco_max_hz = QUAD_VCO_HW_MAX_HZ;
  cfg->clk_sys_nominal_hz = 133000000ULL;

  cfg->if_shift_hz = 11000u;
  cfg->nco_max_hz = 62500u;
  cfg->nco_sweet_lo = 9600u;
  cfg->nco_sweet_hi = 14500u;
  cfg->nco_forbid_lo = 0u;
  cfg->nco_forbid_hi = 5000u;
}

// ---------------------------------------------------------------------------
bool quad_vfo_init(const quad_vfo_cfg_t *cfg) {
  if (!cfg) return false;
  if (!pio_ok(cfg->pio)) return false;
  if (cfg->pin_i + 1 > 29) return false;
  if (cfg->sm < -1 || cfg->sm > 3) return false;
  if (cfg->xosc_hz == 0) return false;
  if (cfg->clk_sys_min_hz == 0) return false;

  if (cfg->nco_max_hz == 0 &&
      (cfg->nco_sweet_lo != 0 || cfg->nco_sweet_hi != 0 ||
       cfg->nco_forbid_lo != 0 || cfg->nco_forbid_hi != 0)) return false;
  if (cfg->nco_sweet_lo > cfg->nco_sweet_hi) return false;
  if (cfg->nco_forbid_lo > cfg->nco_forbid_hi) return false;
  if (cfg->nco_max_hz != 0 && cfg->if_shift_hz >= cfg->nco_max_hz)
    return false;
  if (cfg->nco_max_hz == 0 && cfg->if_shift_hz != 0) return false;

  // Пользовательский потолок VCO — только sanity-границы. Выше  
  // даташитных 1600 МГц не запрещаем: захват и частота всё равно  
  // подтверждаются LOCK + измерением в quad_vfo_set_clk_sys().  
  if (cfg->pll_vco_max_hz < QUAD_VCO_HW_MIN_HZ) return false;
  if (!pll_profile_ok(&cfg->pll_nominal, cfg->xosc_hz)) return false;
  if (!pll_profile_ok(&cfg->pll_ceiling, cfg->xosc_hz)) return false;
  if (!pll_profile_consistent(&cfg->pll_nominal, cfg->xosc_hz)) return false;
  if (!pll_profile_consistent(&cfg->pll_ceiling, cfg->xosc_hz)) return false;

  bool same_sm = (s_sm_claimed && s_cfg.pio == cfg->pio &&
                  cfg->sm >= 0 && (unsigned)cfg->sm == s_sm);
  int target_sm = -1;
  if (cfg->sm >= 0) {
    if (!same_sm && pio_sm_is_claimed(cfg->pio, (uint)cfg->sm)) return false;
    target_sm = cfg->sm;
  } else {
    for (int cand = 0; cand < 4; cand++) {
      bool mine = (s_sm_claimed && s_cfg.pio == cfg->pio &&
                   s_sm == (unsigned)cand);
      if (mine || !pio_sm_is_claimed(cfg->pio, (uint)cand)) {
        target_sm = cand;
        break;
      }
    }
    if (target_sm < 0) return false;
  }

  int idx = (int)pio_get_index(cfg->pio);
  if (s_offset[idx] < 0 &&
      !pio_can_add_program(cfg->pio, &pio_quad_program)) return false;

  if (s_offset[idx] < 0)
    s_offset[idx] = (int)pio_add_program(cfg->pio, &pio_quad_program);
  const unsigned new_offset = (unsigned)s_offset[idx];

  const bool reuse_sm = s_sm_claimed &&
                        s_cfg.pio == cfg->pio &&
                        s_sm == (unsigned)target_sm;
  if (!reuse_sm) pio_sm_claim(cfg->pio, (uint)target_sm);

  if (s_sm_claimed) {
    quad_vfo_stop();
    if (!reuse_sm) pio_sm_unclaim(s_cfg.pio, s_sm);
  }

  s_cfg = *cfg;
  s_have_cfg = true;
  s_sm = (unsigned)target_sm;
  s_sm_claimed = true;
  s_cur_offset = new_offset;
  return true;
}

// ---------------------------------------------------------------------------
bool quad_vfo_init_simple(PIO pio, uint8_t pin_i) {
  quad_vfo_cfg_t c;
  quad_vfo_default_cfg(&c);
  c.pio = pio;
  c.pin_i = pin_i;
  c.sm = -1;
  c.if_shift_hz = 0;
  c.nco_sweet_lo = 0;
  c.nco_sweet_hi = 0;
  c.nco_forbid_lo = 0;
  c.nco_forbid_hi = 0;
  c.nco_max_hz = 0;
  return quad_vfo_init(&c);
}

// ---------------------------------------------------------------------------
static bool div_nco_ok(uint64_t clk_sys_hz, uint32_t lo_hz,
                       uint32_t div_q8, int64_t *out_f_nco) {
  if (clk_sys_hz == 0 || div_q8 == 0) return false;

  const uint64_t lo_got = (clk_sys_hz * 256ull) /
                          (4ull * (uint64_t)div_q8);
  const int64_t f_nco = (s_cfg.nco_max_hz != 0)
      ? (int64_t)((uint64_t)lo_hz + s_cfg.if_shift_hz) - (int64_t)lo_got
      : (int64_t)lo_hz - (int64_t)lo_got;
  const uint64_t abs_hz = (uint64_t)(f_nco < 0 ? -f_nco : f_nco);
  bool ok;
  if (s_cfg.nco_max_hz == 0) {
    const uint64_t q = div_q8;
    const uint64_t den = q * (q + 1ull);
    const uint64_t tol = den
        ? ((clk_sys_hz * 32ull + den - 1ull) / den) + 1ull : 0ull;
    ok = abs_hz <= tol;
  } else {
    ok = f_nco > 0 &&
         f_nco < (int64_t)s_cfg.nco_max_hz &&
         !(f_nco >= (int64_t)s_cfg.nco_forbid_lo &&
           f_nco <= (int64_t)s_cfg.nco_forbid_hi);
  }
  if (out_f_nco) *out_f_nco = f_nco;
  return ok;
}

// ---------------------------------------------------------------------------
static bool recompute_div(uint64_t clk_sys_hz, uint32_t lo_hz,
                          uint32_t *out_div_q8, int64_t *out_f_nco,
                          bool *out_is_int) {
  if (clk_sys_hz == 0 || lo_hz == 0) return false;
  const uint64_t q_min_hw = 2ull << 8;
  const uint64_t q_max_hw = 0xFFFFull << 8;
  uint64_t q_lo, q_hi;

  if (s_cfg.nco_max_hz != 0) {
    const uint64_t lo_t = (uint64_t)lo_hz + s_cfg.if_shift_hz;
    const uint64_t got_hi = lo_t;
    const uint64_t got_lo = (lo_t > s_cfg.nco_max_hz)
                            ? lo_t - s_cfg.nco_max_hz : 1ull;
    q_lo = (clk_sys_hz * 64ull) / got_hi + 1ull;
    q_hi = (clk_sys_hz * 64ull + got_lo - 1ull) / got_lo;
  } else {
    uint64_t q0 = (clk_sys_hz * 256ull + 2ull * lo_hz) / (4ull * lo_hz);
    q_lo = (q0 > q_min_hw + 4) ? q0 - 4 : q_min_hw;
    q_hi = q0 + 4;
  }
  q_lo = (q_lo > q_min_hw + 2) ? q_lo - 2 : q_min_hw;
  q_hi += 2;
  if (q_hi > q_max_hw) q_hi = q_max_hw;
  if (q_lo > q_hi) return false;

  uint64_t best_cost = UINT64_MAX;
  uint32_t best_div = 0;
  int64_t best_nco = 0;
  bool best_int = true;
  for (uint64_t q = q_lo; q <= q_hi; q++) {
    uint32_t dq = (uint32_t)q;
    int64_t f_nco;
    if (!div_nco_ok(clk_sys_hz, lo_hz, dq, &f_nco)) continue;

    uint64_t cost;
    if (s_cfg.nco_max_hz != 0) {
      if (f_nco < (int64_t)s_cfg.nco_sweet_lo)
        cost = (uint64_t)((int64_t)s_cfg.nco_sweet_lo - f_nco);
      else if (f_nco > (int64_t)s_cfg.nco_sweet_hi)
        cost = (uint64_t)(f_nco - (int64_t)s_cfg.nco_sweet_hi);
      else
        cost = 0;
    } else {
      cost = (uint64_t)(f_nco < 0 ? -f_nco : f_nco);
    }
    bool is_int = ((dq & 0xFFu) == 0);
    if (cost < best_cost || (cost == best_cost && is_int && !best_int)) {
      best_cost = cost;
      best_div = dq;
      best_nco = f_nco;
      best_int = is_int;
    }
    if (cost == 0 && is_int) break;
  }
  if (best_div == 0) return false;
  *out_div_q8 = best_div;
  *out_f_nco = best_nco;
  *out_is_int = best_int;
  return true;
}

// ---------------------------------------------------------------------------
bool quad_vfo_start(uint32_t freq_hz) {  
  if (!s_have_cfg) return false;  
  if (freq_hz <= s_cfg.if_shift_hz) return false;  
  
  uint32_t lo_hz = freq_hz - s_cfg.if_shift_hz;  
  if (lo_hz > QUAD_VFO_LO_MAX_HZ) {  
    QV_LOG("[QUAD] ОШИБКА: LO=%lu выше предела %lu Гц — "  
                  "запуск отменён\n",  
                  (unsigned long)lo_hz,  
                  (unsigned long)QUAD_VFO_LO_MAX_HZ);  
    return false;  
  }  
  QV_LOG("[QUAD] Целевая частота LO %lu\n", (unsigned long)lo_hz);

  const uint64_t xosc_hz = s_cfg.xosc_hz;
  uint64_t clk_max = (s_cfg.clk_sys_max_hz < 420000000ULL)
                     ? s_cfg.clk_sys_max_hz : 420000000ULL;
  if (clk_max < s_cfg.clk_sys_min_hz) {
    clk_max = s_cfg.pll_ceiling.clk_sys_hz;
    QV_LOG("[QUAD] выполнен разгон: потолок clk_sys_max_hz=%lu МГц"
                  " (паспорт %lu МГц)\n",
                  (unsigned long)(clk_max / 1000000ULL),
                  (unsigned long)(s_cfg.clk_sys_max_hz / 1000000ULL));
  }
  const uint64_t vco_max = s_cfg.pll_vco_max_hz;
  const uint64_t W2 = 1000000000ull;
  const uint64_t W3 = 1000000ull;
  const uint64_t W4 = 1000ull;
  const uint64_t W5 = 1ull;

  quad_vfo_pll_t best = s_cfg.pll_nominal;
  uint64_t best_cost = UINT64_MAX;
  uint32_t best_div_q8 = 0;
  bool best_is_int = true;
  bool best_legal = false;

  quad_vfo_pll_t fb_best = s_cfg.pll_nominal;
  uint64_t fb_bad = UINT64_MAX;
  uint32_t fb_div_q8 = 0;
  bool fb_is_int = true;

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
              if (D < 2) D = 2;
              if (D > 0xFFFF) D = 0xFFFF;
              div_q8 = (uint32_t)(D << 8);
            } else {
              uint64_t q = (clk * 256ull + 2ull * lo_hz) / (4ull * lo_hz);
              if (q < (2u << 8)) q = 2u << 8;
              if (q > (0xFFFFu << 8)) q = 0xFFFFu << 8;
              div_q8 = (uint32_t)q;
            }
            uint64_t lo_got = (clk * 256ull) / (4ull * (uint64_t)div_q8);
            if (s_cfg.nco_max_hz != 0 && lo_got >= freq_hz) continue;
            int64_t f_nco = (int64_t)freq_hz - (int64_t)lo_got;
            if (s_cfg.nco_max_hz != 0 &&
                f_nco >= (int64_t)s_cfg.nco_max_hz) continue;

            if (s_cfg.nco_max_hz != 0 &&
                f_nco >= (int64_t)s_cfg.nco_forbid_lo &&
                f_nco <= (int64_t)s_cfg.nco_forbid_hi) {
              uint64_t bad = (uint64_t)((int64_t)s_cfg.nco_forbid_hi - f_nco);
              if (bad < fb_bad) {
                fb_bad = bad;
                fb_best.refdiv = refdiv; fb_best.p1 = p1; fb_best.p2 = p2;
                fb_best.fbdiv = fbdiv; fb_best.clk_sys_hz = clk;
                fb_best.vsel = quad_vfo_vsel_for(clk);
                fb_best.is_oc = (clk > s_cfg.clk_sys_nominal_hz);
                fb_div_q8 = div_q8;
                fb_is_int = (frac_mode == 0);
              }
              continue;
            }

            uint64_t cost = 0;
            if (s_cfg.nco_max_hz == 0) {
              cost += (uint64_t)(f_nco < 0 ? -f_nco : f_nco) * W2;
            } else {
              if (f_nco < (int64_t)s_cfg.nco_sweet_lo)
                cost += (uint64_t)((int64_t)s_cfg.nco_sweet_lo - f_nco) * W2;
              else if (f_nco > (int64_t)s_cfg.nco_sweet_hi)
                cost += (uint64_t)(f_nco - (int64_t)s_cfg.nco_sweet_hi) * W2;
            }
            bool is_int = (frac_mode == 0);
            if (!is_int) {
              cost += W3;
              uint32_t frac = div_q8 & 0xFF;
              if (frac) {
                int32_t m = (int32_t)frac % 32;
                int32_t dist = (m < 16) ? m : 32 - m;
                cost += (uint64_t)(16 - dist) * W4;
              }
            }
            cost += ((clk_max - clk) / 1000000ull) * W5;

            if (cost < best_cost) {
              best_cost = cost;
              best_div_q8 = div_q8;
              best_is_int = is_int;
              best_legal = true;
              best.refdiv = refdiv; best.p1 = p1; best.p2 = p2;
              best.fbdiv = fbdiv; best.clk_sys_hz = clk;
              best.vsel = quad_vfo_vsel_for(clk);
              best.is_oc = (clk > s_cfg.clk_sys_nominal_hz);
            }
          }
        }
      }
    }
  }

  if (!best_legal && fb_bad == UINT64_MAX) {
    Serial.println("[QUAD] ОШИБКА: скан не дал ни одного кандидата — "
                   "проверьте clk_sys_max_hz/pll_ceiling. LO не тронут.");
    return false;
  }
  if (!best_legal) {
    Serial.println("[QUAD] ВНИМАНИЕ: нет кандидата с дыркой вне "
                   "запретной зоны — ставлю наименьшее зло");
    best = fb_best;
    best_div_q8 = fb_div_q8;
    best_is_int = fb_is_int;
  }

  // === Точка невозврата: гасим SM и перепрошиваем шину ===
  pio_sm_set_enabled(s_cfg.pio, s_sm, false);

  uint32_t vsel = best.vsel;
  if (vsel == 0 || vsel < (uint32_t)VREG_VOLTAGE_1_10)
    vsel = quad_vfo_vsel_for(best.clk_sys_hz);

  // Три исхода: цель подтверждена / номинал восстановлен / отказ
  bool nominal_ok = false;
  bool pll_ok = quad_vfo_set_clk_sys(&best, vsel,
                                     &s_cfg.pll_nominal, xosc_hz,
                                     &nominal_ok);

  uint64_t clk_sys_hz = clock_get_hz(clk_sys);
  uint32_t div_q8;
  int64_t f_nco;
  bool is_int;

  if ((!pll_ok && !nominal_ok) || clk_sys_hz == 0) {
    Serial.println("[QUAD] ОШИБКА: целевой и номинальный PLL "
                   "не подтверждены — генератор не запущен");
    fail_path();
    return false;
  }
  if (!pll_ok && nominal_ok) {
    QV_LOG("[QUAD] ВНИМАНИЕ: целевой PLL не подтверждён; "
                  "используется восстановленный номинальный профиль "
                  "(clk_sys=%llu Hz)\n",
                  (unsigned long long)clk_sys_hz);
  }

  if (clk_sys_hz != best.clk_sys_hz) {
    uint64_t diff = (clk_sys_hz > best.clk_sys_hz)
        ? clk_sys_hz - best.clk_sys_hz : best.clk_sys_hz - clk_sys_hz;
    QV_LOG("[QUAD] ВНИМАНИЕ: шина %llu Hz вместо целевых %lu "
                  "(расхождение %llu Hz) — пересчёт делителя\n",
                  (unsigned long long)clk_sys_hz,
                  (unsigned long)best.clk_sys_hz,
                  (unsigned long long)diff);
  }

  if (div_nco_ok(clk_sys_hz, lo_hz, best_div_q8, &f_nco)) {
    div_q8 = best_div_q8;
    is_int = best_is_int;
  } else if (recompute_div(clk_sys_hz, lo_hz, &div_q8, &f_nco, &is_int)) {
    Serial.println("[QUAD] делитель сканера не подошёл — "
                   "подобран новый под фактическую шину");
  } else {
    Serial.println("[QUAD] ОШИБКА: на фактической шине ни один "
                   "делитель не удовлетворяет NCO-ограничениям");
    fail_path();
    return false;
  }

  // Фиксируем применённый профиль PLL для обратного вывода.  
  {  
    const quad_vfo_pll_t *applied = pll_ok ? &best : &s_cfg.pll_nominal;  
    s_pll_refdiv = (uint16_t)applied->refdiv;  
    s_pll_fbdiv  = (uint16_t)applied->fbdiv;  
    s_pll_p1     = (uint8_t)applied->p1;  
    s_pll_p2     = (uint8_t)applied->p2;  
    s_pll_nominal_used = !pll_ok;  
  }  
  
  {
    uint64_t lo_got = (clk_sys_hz * 256ull) / (4ull * (uint64_t)div_q8);
    s_lo_actual_hz = (uint32_t)lo_got;
    s_lo_error_hz = (int32_t)((int64_t)lo_hz - (int64_t)lo_got);
  }
  
  {  
    bool sweet = (s_cfg.nco_max_hz != 0 &&  
                  f_nco >= (int64_t)s_cfg.nco_sweet_lo &&  
                  f_nco <= (int64_t)s_cfg.nco_sweet_hi);  
    uint64_t vco_hz = (s_pll_refdiv)  
        ? (xosc_hz / s_pll_refdiv) * s_pll_fbdiv : 0;  
    QV_LOG("[QUAD] clk_sys=%llu Hz  PLL refdiv=%u fbdiv=%u "  
                  "p1=%u p2=%u VCO=%llu Hz%s  "  
                  "clkdiv Q8=0x%lX (%s)  LO=%lu Hz  err=%ld Hz  "  
                  "f_nco=%ld sweet=%d\n",  
                  (unsigned long long)clk_sys_hz,  
                  (unsigned)s_pll_refdiv, (unsigned)s_pll_fbdiv,  
                  (unsigned)s_pll_p1, (unsigned)s_pll_p2,  
                  (unsigned long long)vco_hz,  
                  s_pll_nominal_used ? " (nominal)" : "",  
                  (unsigned long)div_q8,  
                  is_int ? "целый" : "дробный",  
                  (unsigned long)s_lo_actual_hz,  
                  (long)s_lo_error_hz,  
                  (long)f_nco, sweet ? 1 : 0);  
  }

  // === Конфигурация SM под квадратуру ===
  pio_sm_config cq = pio_get_default_sm_config();
  sm_config_set_wrap(&cq, s_cur_offset + 0, s_cur_offset + 3);
  sm_config_set_set_pins(&cq, s_cfg.pin_i, 2);
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
  gpio_set_dir(s_cfg.pin_i, GPIO_IN);
  gpio_set_dir(s_cfg.pin_i + 1, GPIO_IN);
  s_active = false;  
  s_lo_actual_hz = 0;  
  s_lo_error_hz = 0;  
  s_pll_refdiv = s_pll_fbdiv = 0;  
  s_pll_p1 = s_pll_p2 = 0;  
  s_pll_nominal_used = false;  
}

uint32_t quad_vfo_actual_hz(void) { return s_lo_actual_hz; }
int32_t quad_vfo_error_hz(void) { return s_lo_error_hz; }
bool quad_vfo_active(void) { return s_active; }

// Параметры фактически применённого профиля PLL из последнего  
// успешного старта. До старта / после stop возвращают нули.  
uint16_t quad_vfo_pll_refdiv(void)  { return s_pll_refdiv; }  
uint16_t quad_vfo_pll_fbdiv(void)   { return s_pll_fbdiv; }  
uint8_t  quad_vfo_pll_p1(void)      { return s_pll_p1; }  
uint8_t  quad_vfo_pll_p2(void)      { return s_pll_p2; }  
// true — работаем на восстановленном номинальном профиле (rollback),  
// false — на целевом профиле сканера.  
bool     quad_vfo_pll_nominal(void) { return s_pll_nominal_used; }  

void quad_vfo_set_quiet(bool on) {  
  s_quiet = on;  
  quad_vfo_pll_set_quiet(on);  
}

// VCO применённого профиля в Гц: xosc/refdiv*fbdiv; 0 — не запущен.  
uint64_t quad_vfo_pll_vco_hz(void) {  
  return s_pll_refdiv  
      ? (s_cfg.xosc_hz / s_pll_refdiv) * s_pll_fbdiv : 0;  
}

// ---------------------------------------------------------------------------
void quad_vfo_debug(void) {
  if (!s_have_cfg) {
    Serial.println("[LODBG] не инициализирован");
    return;
  }
  uint32_t meas_clk = quad_vfo_clk_sys_meas_hz();  
  uint32_t cdiv = s_cfg.pio->sm[s_sm].clkdiv;  
  
  QV_LOG("[LODBG] clk_sys: cached=%lu Hz  measured=%lu Hz "  
                "(delta=%ld Hz)\n",  
                (unsigned long)clock_get_hz(clk_sys),  
                (unsigned long)meas_clk,  
                (long)((int64_t)meas_clk - (int64_t)clock_get_hz(clk_sys)));
  QV_LOG("[LODBG] PIO=%d SM=%d offset=%u  clkdiv=0x%08lX "
                "(int=%lu frac=%lu)\n",
                pio_get_index(s_cfg.pio), (int)s_sm, s_cur_offset,
                (unsigned long)cdiv,
                (unsigned long)(cdiv >> 16),
                (unsigned long)((cdiv >> 8) & 0xFF));
  QV_LOG("[LODBG] SM ctrl=0x%08lX enabled=%d  execctrl=0x%08lX\n",
                (unsigned long)s_cfg.pio->ctrl,
                (int)((s_cfg.pio->ctrl & (1u << s_sm)) != 0),
                (unsigned long)s_cfg.pio->sm[s_sm].execctrl);
  QV_LOG("[LODBG] lo_hz=%lu err=%ld Hz active=%d\n",
                (unsigned long)s_lo_actual_hz,
                (long)s_lo_error_hz,
                (int)s_active);
  uint64_t vco_hz = (s_pll_refdiv)  
      ? (s_cfg.xosc_hz / s_pll_refdiv) * s_pll_fbdiv : 0;  
  QV_LOG("[LODBG] PLL: refdiv=%u fbdiv=%u postdiv=%u/%u "  
                "VCO=%llu Hz %s\n",  
                (unsigned)s_pll_refdiv, (unsigned)s_pll_fbdiv,  
                (unsigned)s_pll_p1, (unsigned)s_pll_p2,  
                (unsigned long long)vco_hz,  
                s_pll_nominal_used ? "(nominal fallback)" : "(target)");
}
