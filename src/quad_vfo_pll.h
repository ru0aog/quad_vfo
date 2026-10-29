// quad_vfo_pll.h  
// ============================================================================  
// PLL-подсистема quad_vfo: безопасное перепрограммирование pll_sys + VREG.  
// Порт из src/vfo/soft_tx.cpp (soft_tx_set_clk_sys / soft_tx_vsel_for).  
// Все глобалы убраны: xosc_hz и pll_nominal передаются параметрами.  
// ============================================================================  
#pragma once  
#include <Arduino.h>  
  
typedef struct {  
  uint32_t fbdiv;       // множитель VCO  
  uint32_t p1, p2;      // post-делители  
  uint32_t clk_sys_hz;  // итоговая частота шины, Гц  
  uint32_t vsel;        // VREG_VOLTAGE_*  
  bool     is_oc;       // true — конфигурация разгона  
  uint32_t refdiv;      // делитель опоры PLL (обычно 1)  
} quad_vfo_pll_t;  
  
// VSEL по частоте шины (таблица из soft_tx_vsel_for).  
uint32_t quad_vfo_vsel_for(uint64_t clk_hz);  
  
// Безопасное перепрограммирование pll_sys: detach clk_peri → XOSC →  
// pll_init → LOCK-таймаут → SSI BAUDR → clk_sys → верификация  
// frequency_count_khz с 3 ретраями → откат на nominal при провале.  
// xosc_hz — опора; nominal — конфигурация отката.  
bool quad_vfo_set_clk_sys(const quad_vfo_pll_t *cfg, uint32_t vsel,  
                          const quad_vfo_pll_t *nominal, uint64_t xosc_hz);  
  
// Реальная частота шины по последней успешной прошивке (измеренная).  
uint32_t quad_vfo_clk_sys_hz(void);
