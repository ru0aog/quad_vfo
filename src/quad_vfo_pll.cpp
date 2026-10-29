// quad_vfo_pll.cpp  
// Порт PLL-подсистемы из src/vfo/soft_tx.cpp (RU0AOG) + правки аудита:  
//  - pll_init() SDK исключён: регистры PLL настраиваются напрямую той же  
//    последовательностью (cs -> fbdiv_int -> pwr -> LOCK -> prim ->  
//    postdiv), но с ограниченным ожиданием захвата — зависание внутри  
//    SDK исключено полностью;  
//  - rollback_nominal() возвращает bool и сама управляет прерываниями:  
//    clk_sys НЕ переключается на PLL без подтверждённого LOCK — шина  
//    остаётся на XOSC;  
//  - оба пути отката идут при выключенных прерываниях.  
//  
// NB: последовательность pll_setup_bounded() следует сверить с  
// pll_init() из clocks.c вашей версии pico-sdk при обновлении ядра.  
#include "quad_vfo_pll.h"  
#include "hardware/clocks.h"  
#include "hardware/pll.h"  
#include "hardware/sync.h"  
#include "hardware/timer.h"  
#include "hardware/vreg.h"  
#include "hardware/structs/ssi.h"  
#include "hardware/structs/pll.h"  
  
#define QUAD_LOCK_TIMEOUT  20000u  
  
static uint32_t s_clk_sys_hz = 133000000;  
  
uint32_t quad_vfo_clk_sys_hz(void) { return s_clk_sys_hz; }  
  
uint32_t quad_vfo_vsel_for(uint64_t clk_hz) {  
  if (clk_hz >= 250000000ULL)      return VREG_VOLTAGE_1_30;  
  else if (clk_hz >= 200000000ULL) return VREG_VOLTAGE_1_25;  
  else if (clk_hz >  133000000ULL) return VREG_VOLTAGE_1_15;  
  return VREG_VOLTAGE_DEFAULT;  
}  
  
// ---------------------------------------------------------------------------  
// Ожидание PLL LOCK с таймаутом. true — захвачен.  
// ---------------------------------------------------------------------------  
static inline bool wait_pll_lock(void) {  
  uint32_t t = QUAD_LOCK_TIMEOUT;  
  while (!(pll_sys_hw->cs & PLL_CS_LOCK_BITS) && --t);  
  return (t != 0);  
}  
  
// ---------------------------------------------------------------------------  
// Настройка pll_sys регистрами напрямую, с ограниченным ожиданием LOCK.  
// Повторяет pll_init() pico-sdk:  
//   cs = refdiv; fbdiv_int = vco/ref; pwr on; LOCK; prim = p1<<16|p2<<8;  
//   POSTDIVPD off.  
// false — LOCK не захвачен за QUAD_LOCK_TIMEOUT; PLL при этом остаётся  
// выключенным/незахваченным, clk_sys не трогается (вызывающий — на XOSC).  
// ---------------------------------------------------------------------------  
static bool __not_in_flash_func(pll_setup_bounded)(uint32_t refdiv,  
                                                   uint32_t fbdiv,  
                                                   uint32_t p1,  
                                                   uint32_t p2) {  
  // Сброс делителей и отключение питания перед перенастройкой.  
  pll_sys_hw->cs = refdiv;  
  pll_sys_hw->fbdiv_int = fbdiv;  
  
  // Включение PLL и VCO (сброс PD/VCOPD), POSTDIV пока выключен.  
  hw_clear_bits(&pll_sys_hw->pwr,  
                PLL_PWR_PD_BITS | PLL_PWR_VCOPD_BITS | PLL_PWR_POSTDIVPD_BITS);  
  
  if (!wait_pll_lock()) {  
    // Не захватился — выключаем обратно, чтобы следующая попытка  
    // начиналась с чистого состояния.  
    hw_set_bits(&pll_sys_hw->pwr,  
                PLL_PWR_PD_BITS | PLL_PWR_VCOPD_BITS | PLL_PWR_POSTDIVPD_BITS);  
    return false;  
  }  
  
  // Post-делители и выход на clk_sys.  
  pll_sys_hw->prim = (p1 << PLL_PRIM_POSTDIV1_LSB) |  
                     (p2 << PLL_PRIM_POSTDIV2_LSB);  
  hw_clear_bits(&pll_sys_hw->pwr, PLL_PWR_POSTDIVPD_BITS);  
  return true;  
}  
  
// ---------------------------------------------------------------------------  
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
                  (uint32_t)xosc_hz, (uint32_t)xosc_hz);  
}  
  
// clk_sys -> PLL_SYS  
static void __not_in_flash_func(clk_sys_to_pll)(uint32_t clk_hz) {  
  clock_configure(clk_sys,  
                  CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,  
                  CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,  
                  clk_hz, clk_hz);  
}  
  
// ---------------------------------------------------------------------------  
// Откат на номинал. Управляет прерываниями САМА — безопасно вызывать и  
// из секции с выключенными прерываниями, и снаружи.  
// true  — шина на nominal, s_clk_sys_hz = nominal->clk_sys_hz;  
// false — nominal тоже не захватил LOCK: шина остаётся на XOSC,  
//         s_clk_sys_hz = xosc_hz.  
// ---------------------------------------------------------------------------  
static bool __not_in_flash_func(rollback_nominal)(  
        const quad_vfo_pll_t *nominal, uint64_t xosc_hz) {  
  uint32_t ints = save_and_disable_interrupts();  
  clk_sys_to_xosc(xosc_hz);  
  bool ok = pll_setup_bounded(nominal->refdiv, nominal->fbdiv,  
                              nominal->p1, nominal->p2);  
  if (ok) {  
    clk_sys_to_pll(nominal->clk_sys_hz);  
  }  
  restore_interrupts(ints);  
  
  s_clk_sys_hz = ok ? nominal->clk_sys_hz : (uint32_t)xosc_hz;  
  return ok;  
}  
  
// ---------------------------------------------------------------------------  
bool __not_in_flash_func(quad_vfo_set_clk_sys)(  
        const quad_vfo_pll_t *cfg_in, uint32_t vsel,  
        const quad_vfo_pll_t *nominal, uint64_t xosc_hz) {  
  quad_vfo_pll_t cfg = *cfg_in;  
  if (cfg.refdiv == 0) cfg.refdiv = 1;  
  
  // p1=1 недокументированно делит пополам на VCO < ~2.8 ГГц — отклоняем.  
  if (cfg.p1 < 2) cfg = *nominal;  
  
  uint32_t target_clk_hz = (uint32_t)(((uint64_t)cfg.fbdiv * xosc_hz)  
                                    / (uint64_t)(cfg.refdiv * cfg.p1 * cfg.p2));  
  bool is_overclocking = (target_clk_hz > s_clk_sys_hz);  
  
  if (is_overclocking) {  
    vreg_set_voltage((vreg_voltage)vsel);  
    busy_wait_us(500);  
  }  
  
  detach_peri();  
  uint32_t ints_status = save_and_disable_interrupts();  
  
  clk_sys_to_xosc(xosc_hz);  
  
  // Прямая настройка регистров с контролируемым LOCK.  
  // Округление ВВЕРХ до кратного ref_hz сохраняет семантику исходника.  
  uint64_t ref_hz = (xosc_hz + cfg.refdiv - 1u) / cfg.refdiv;  
  uint32_t fbdiv_calc = (uint32_t)((uint64_t)cfg.fbdiv);   // fbdiv уже целый  
  (void)ref_hz;  
  bool pll_ok = pll_setup_bounded(cfg.refdiv, fbdiv_calc, cfg.p1, cfg.p2);  
  
  if (!pll_ok) {  
    // Целевой PLL не захватился — откат в той же секции прерываний  
    // (rollback_nominal сам делает save/restore и корректно вложится).  
    clk_sys_to_xosc(xosc_hz);  
    bool nom_ok = rollback_nominal(nominal, xosc_hz);  
    restore_interrupts(ints_status);  
    if (!nom_ok)  
      Serial.println("[PLL] КРИТИЧНО: откат на номинал не удался — "  
                     "шина на XOSC");  
    return false;  
  }  
  
  // SSI BAUDR: такт флэша <= 48 МГц, чётный делитель 2..34.  
  const uint32_t FLASH_SAFE_MAX_HZ = 48000000u;  
  uint32_t ssi_baud = (uint32_t)((target_clk_hz + FLASH_SAFE_MAX_HZ - 1)  
                               / FLASH_SAFE_MAX_HZ);  
  ssi_baud = (ssi_baud + 1u) & ~1u;  
  if (ssi_baud < 2u)  ssi_baud = 2u;  
  if (ssi_baud > 34u) ssi_baud = 34u;  
  ssi_hw->ssienr = 0;  
  ssi_hw->baudr  = ssi_baud;  
  ssi_hw->ssienr = 1;  
  
  clk_sys_to_pll(target_clk_hz);  
  restore_interrupts(ints_status);  
  
  // Самопроверка: до 3 попыток, каждая — полный цикл с LOCK-контролем.  
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
    bool ok2 = pll_setup_bounded(cfg.refdiv, fbdiv_calc, cfg.p1, cfg.p2);  
    if (ok2) clk_sys_to_pll(target_clk_hz);  
    restore_interrupts(ints2);  
    if (!ok2) break;   // PLL не захватился — дальше бессмысленно  
  }  
  
  if (!locked_ok) {  
    // 3 промаха — откат на номинал. rollback_nominal сама отключает  
    // прерывания и проверяет LOCK — согласованно с первым путём.  
    bool nom_ok = rollback_nominal(nominal, xosc_hz);  
    if (!nom_ok)  
      Serial.println("[PLL] КРИТИЧНО: откат на номинал не удался — "  
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