// quad_vfo_pll.cpp  
// Порт PLL-подсистемы из src/vfo/soft_tx.cpp (GPLv3, RU0AOG).  
#include "quad_vfo_pll.h"  
#include "hardware/clocks.h"  
#include "hardware/pll.h"  
#include "hardware/sync.h"  
#include "hardware/timer.h"  
#include "hardware/vreg.h"  
#include "hardware/structs/ssi.h"  
#include "hardware/structs/pll.h"  
  
static uint32_t s_clk_sys_hz = 133000000;   // кэш реальной шины  
  
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
  
bool __not_in_flash_func(quad_vfo_set_clk_sys)(const quad_vfo_pll_t *cfg_in,  
                                             uint32_t vsel,  
                                             const quad_vfo_pll_t *nominal,  
                                             uint64_t xosc_hz) {  
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
  
  // clk_sys -> XOSC  
  clock_configure(clk_sys,  
                  CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,  
                  CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_XOSC_CLKSRC,  
                  xosc_hz, xosc_hz);  
  
  // Округление ВВЕРХ до кратного ref_hz.  
  uint64_t ref_hz = (xosc_hz + cfg.refdiv - 1u) / cfg.refdiv;  
  uint32_t vco_nominal_hz = (uint32_t)(ref_hz * (uint64_t)cfg.fbdiv);  
  
  pll_init(pll_sys, cfg.refdiv, vco_nominal_hz, cfg.p1, cfg.p2);  
  
  uint32_t timeout = 20000;  
  while (!(pll_sys_hw->cs & PLL_CS_LOCK_BITS) && --timeout);  
  
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
  
  // clk_sys -> PLL_SYS  
  clock_configure(clk_sys,  
                  CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,  
                  CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,  
                  target_clk_hz, target_clk_hz);  
  
  restore_interrupts(ints_status);  
  
  // Самопроверка: при промахе повторяем ВЕСЬ цикл до 3 раз.  
  bool locked_ok = false;  
  uint32_t meas_khz = 0;  
  uint32_t expect_khz = cfg.clk_sys_hz / 1000u;  
  
  for (int attempt = 0; attempt < 3; attempt++) {  
    busy_wait_us(300);  
    meas_khz = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS);  
    if (meas_khz > expect_khz * 997u / 1000u &&  
        meas_khz < expect_khz * 1003u / 1000u) {  
      locked_ok = true;  
      break;  
    }  
    uint32_t ints2 = save_and_disable_interrupts();  
    clock_configure(clk_sys,  
                    CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,  
                    CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_XOSC_CLKSRC,  
                    xosc_hz, xosc_hz);  
    pll_init(pll_sys, cfg.refdiv, vco_nominal_hz, cfg.p1, cfg.p2);  
    timeout = 20000;  
    while (!(pll_sys_hw->cs & PLL_CS_LOCK_BITS) && --timeout);  
    clock_configure(clk_sys,  
                    CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,  
                    CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,  
                    target_clk_hz, target_clk_hz);  
    restore_interrupts(ints2);  
  }  
  
  if (!locked_ok) {  
    // Откат на номинал.  
    clock_configure(clk_sys,  
                    CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,  
                    CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_XOSC_CLKSRC,  
                    xosc_hz, xosc_hz);  
    uint64_t ref_fb = (xosc_hz + nominal->refdiv - 1u) / nominal->refdiv;  
    pll_init(pll_sys, nominal->refdiv,  
             (uint32_t)(ref_fb * (uint64_t)nominal->fbdiv),  
             nominal->p1, nominal->p2);  
    timeout = 20000;  
    while (!(pll_sys_hw->cs & PLL_CS_LOCK_BITS) && --timeout);  
    clock_configure(clk_sys,  
                    CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,  
                    CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,  
                    nominal->clk_sys_hz, nominal->clk_sys_hz);  
    s_clk_sys_hz = nominal->clk_sys_hz;  
    return false;  
  }  
  
  if (!is_overclocking) {  
    busy_wait_us(100);  
    vreg_set_voltage((vreg_voltage)vsel);  
  }  
  s_clk_sys_hz = meas_khz * 1000u;   // реальность, а не желаемое  
  return true;  
}
