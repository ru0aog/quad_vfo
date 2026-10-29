// quad_vfo_pll.cpp  
// Порт PLL-подсистемы из src/vfo/soft_tx.cpp (RU0AOG) + правки аудита:  
//  - rollback_nominal() возвращает bool: clk_sys НЕ переключается на  
//    PLL без подтверждённого LOCK — шина остаётся на XOSC;  
//  - все таймауты LOCK после pll_init проверяются;  
//  - опциональный QUAD_VFO_BOUNDED_PLL: собственная настройка регистров  
//    PLL с ограниченным ожиданием захвата (защита и от зависания внутри  
//    штатного pll_init, который ждёт LOCK бесконечно).  
#include "quad_vfo_pll.h"  
#include "hardware/clocks.h"  
#include "hardware/pll.h"  
#include "hardware/sync.h"  
#include "hardware/timer.h"  
#include "hardware/vreg.h"  
#include "hardware/structs/ssi.h"  
#include "hardware/structs/pll.h"  
  
#define QUAD_LOCK_TIMEOUT   20000u     // итераций опроса PLL_CS_LOCK  
  
static uint32_t s_clk_sys_hz = 133000000;  
  
uint32_t quad_vfo_clk_sys_hz(void) { return s_clk_sys_hz; }  
  
uint32_t quad_vfo_vsel_for(uint64_t clk_hz) {  
  if (clk_hz >= 250000000ULL)      return VREG_VOLTAGE_1_30;  
  else if (clk_hz >= 200000000ULL) return VREG_VOLTAGE_1_25;  
  else if (clk_hz >  133000000ULL) return VREG_VOLTAGE_1_15;  
  return VREG_VOLTAGE_DEFAULT;  
}  
  
static void __not_in_flash_func(detach_peri)(void) {  
  clock_configure(clk_peri, 0,  
                  CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,  
                  48 * 1000000, 48 * 1000000);  
}  
  
// clk_sys -> XOSC (безопасная опора для переключения)  
static void __not_in_flash_func(clk_sys_to_xosc)(uint64_t xosc_hz) {  
  clock_configure(clk_sys,  
                  CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,  
                  CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_XOSC_CLKSRC,  
                  xosc_hz, xosc_hz);  
}  
  
// clk_sys -> PLL_SYS на целевой частоте  
static void __not_in_flash_func(clk_sys_to_pll)(uint32_t target_hz) {  
  clock_configure(clk_sys,  
                  CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,  
                  CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,  
                  target_hz, target_hz);  
}  
  
// Ожидание LOCK с таймаутом. true — захвачен.  
static inline bool __not_in_flash_func(wait_pll_lock)(void) {  
  uint32_t t = QUAD_LOCK_TIMEOUT;  
  while (!(pll_sys_hw->cs & PLL_CS_LOCK_BITS) && --t);  
  return (t != 0);  
}  
  
#ifdef QUAD_VFO_BOUNDED_PLL  
// ---------------------------------------------------------------------------  
// Собственная настройка pll_sys БЕЗ pll_init() — каждый шаг ограничен  
// таймаутом. Эквивалент pll_init(refdiv, vco_hz, p1, p2): программирует  
// CS/refdiv, FB фидбэк-делитель, включает PLL и postdiv, ждёт LOCK.  
// true — LOCK захвачен за отведённое время.  
// ---------------------------------------------------------------------------  
static bool __not_in_flash_func(pll_setup_bounded)(uint32_t refdiv,  
                                                   uint32_t vco_hz,  
                                                   uint32_t fbdiv_calc,  
                                                   uint32_t p1, uint32_t p2,  
                                                   uint64_t xosc_hz) {  
  (void)vco_hz; (void)fbdiv_calc; (void)xosc_hz;  
  // Выключаем PLL и postdiv перед изменением параметров  
  pll_sys_hw->pwr = PLL_PWR_PD_BITS | PLL_PWR_VCOPD_BITS |  
                    PLL_PWR_POSTDIVPD_BITS;  
  
  // refdiv  
  hw_write_masked(&pll_sys_hw->cs, refdiv << PLL_CS_REFDIV_LSB,  
                  PLL_CS_REFDIV_BITS);  
  
  // fbdiv  
  pll_sys_hw->fbdiv_int = fbdiv_calc;  
  
  // Включаем PLL (снимаем PD и VCOPD)  
  hw_clear_bits(&pll_sys_hw->pwr, PLL_PWR_PD_BITS | PLL_PWR_VCOPD_BITS);  
  
  if (!wait_pll_lock()) return false;   // <-- контролируемое ожидание  
  
  // Post-делители, затем включение postdiv  
  pll_sys_hw->prim = (p1 << PLL_PRIM_POSTDIV1_LSB) |  
                     (p2 << PLL_PRIM_POSTDIV2_LSB);  
  hw_clear_bits(&pll_sys_hw->pwr, PLL_PWR_POSTDIVPD_BITS);  
  return true;  
}  
#endif // QUAD_VFO_BOUNDED_PLL  
  
// Настройка PLL и ожидание LOCK. true — LOCK подтверждён.  
// По умолчанию использует pll_init() SDK (внутри ждёт LOCK БЕСКОНЕЧНО —  
// внешний таймаут после него контролирует только наш доп. цикл).  
static bool __not_in_flash_func(pll_setup)(uint32_t refdiv,  
                                           uint32_t fbdiv, uint32_t p1,  
                                           uint32_t p2, uint64_t xosc_hz) {  
#ifdef QUAD_VFO_BOUNDED_PLL  
  return pll_setup_bounded(refdiv, 0, fbdiv, p1, p2, xosc_hz);  
#else  
  uint64_t ref_hz = (xosc_hz + refdiv - 1u) / refdiv;  
  uint32_t vco_nominal_hz = (uint32_t)(ref_hz * (uint64_t)fbdiv);  
  pll_init(pll_sys, refdiv, vco_nominal_hz, p1, p2);  
  return wait_pll_lock();  
#endif  
}  
  
// Откат на номинальный профиль.  
// true  — номинал подтверждён, шина на pll_sys.  
// false — номинальный LOCK не захвачен: clk_sys НЕ переключается,  
//         шина остаётся на XOSC, s_clk_sys_hz = xosc_hz.  
static bool __not_in_flash_func(rollback_nominal)(const quad_vfo_pll_t *nominal,  
                                                  uint64_t xosc_hz) {  
  clk_sys_to_xosc(xosc_hz);  
  if (!pll_setup(nominal->refdiv, nominal->fbdiv,  
                 nominal->p1, nominal->p2, xosc_hz)) {  
    s_clk_sys_hz = (uint32_t)xosc_hz;   // система живёт на опоре  
    return false;  
  }  
  clk_sys_to_pll(nominal->clk_sys_hz);  
  s_clk_sys_hz = nominal->clk_sys_hz;  
  return true;  
}  
  
bool __not_in_flash_func(quad_vfo_set_clk_sys)(const quad_vfo_pll_t *cfg_in,  
                                             uint32_t vsel,  
                                             const quad_vfo_pll_t *nominal,  
                                             uint64_t xosc_hz) {  
  quad_vfo_pll_t cfg = *cfg_in;  
  if (cfg.refdiv == 0) cfg.refdiv = 1;  
  
  // p1=1 недокументированно делит пополам на VCO < ~2.8 ГГц — отклоняем.  
  if (cfg.p1 < 2) cfg = *nominal;  
  
  uint32_t target_clk_hz =  
      (uint32_t)(((uint64_t)cfg.fbdiv * xosc_hz) /  
                 (uint64_t)(cfg.refdiv * cfg.p1 * cfg.p2));  
  bool is_overclocking = (target_clk_hz > s_clk_sys_hz);  
  
  if (is_overclocking) {  
    vreg_set_voltage((vreg_voltage)vsel);  
    busy_wait_us(500);  
  }  
  
  detach_peri();  
  uint32_t ints_status = save_and_disable_interrupts();  
  
  clk_sys_to_xosc(xosc_hz);  
  
  if (!pll_setup(cfg.refdiv, cfg.fbdiv, cfg.p1, cfg.p2, xosc_hz)) {  
    // LOCK не захвачен — НЕ переключаем clk_sys на неподтверждённый PLL.  
    bool rb = rollback_nominal(nominal, xosc_hz);  
    restore_interrupts(ints_status);  
    if (!rb)  
      Serial.println("[QUAD-PLL] ОШИБКА: номинальный LOCK не захвачен — "  
                     "шина на XOSC");  
    return false;  
  }  
  
  // SSI BAUDR: такт флэша <= 48 МГц, чётный делитель 2..34.  
  const uint32_t FLASH_SAFE_MAX_HZ = 48000000u;  
  uint32_t ssi_baud =  
      (uint32_t)((target_clk_hz + FLASH_SAFE_MAX_HZ - 1) / FLASH_SAFE_MAX_HZ);  
  ssi_baud = (ssi_baud + 1u) & ~1u;  
  if (ssi_baud < 2u)  ssi_baud = 2u;  
  if (ssi_baud > 34u) ssi_baud = 34u;  
  ssi_hw->ssienr = 0;  
  ssi_hw->baudr  = ssi_baud;  
  ssi_hw->ssienr = 1;  
  
  clk_sys_to_pll(target_clk_hz);  
  restore_interrupts(ints_status);  
  
  // Самопроверка: при промахе повторяем ВЕСЬ цикл до 3 раз.  
  bool locked_ok = false;  
  uint32_t meas_khz = 0;  
  uint32_t expect_khz = (uint32_t)(cfg.clk_sys_hz / 1000u);  
  
  for (int attempt = 0; attempt < 3; attempt++) {  
    busy_wait_us(300);  
    meas_khz = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS);  
    if (meas_khz > expect_khz * 997u / 1000u &&  
        meas_khz < expect_khz * 1003u / 1000u) {  
      locked_ok = true;  
      break;  
    }  
    uint32_t ints2 = save_and_disable_interrupts();  
    clk_sys_to_xosc(xosc_hz);  
    if (!pll_setup(cfg.refdiv, cfg.fbdiv, cfg.p1, cfg.p2, xosc_hz)) {  
      // PLL не захватил даже на ретрае — выходим к откату.  
      restore_interrupts(ints2);  
      break;  
    }  
    clk_sys_to_pll(target_clk_hz);  
    restore_interrupts(ints2);  
  }  
  
  if (!locked_ok) {  
    // 3 промаха — откат на номинал. Если и номинал не захватил —  
    // остаёмся на XOSC (rollback_nominal сам это обрабатывает).  
    bool rb = rollback_nominal(nominal, xosc_hz);  
    if (!rb)  
      Serial.println("[QUAD-PLL] ОШИБКА: номинальный LOCK не захвачен — "  
                     "шина на XOSC");  
    return false;  
  }  
  
  if (!is_overclocking) {  
    busy_wait_us(100);  
    vreg_set_voltage((vreg_voltage)vsel);  
  }  
  s_clk_sys_hz = meas_khz * 1000u;   // реальность, а не желаемое  
  return true;  
}