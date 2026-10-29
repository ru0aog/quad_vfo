// quad_vfo_pll.h  
// ============================================================================  
// PLL-подсистема quad_vfo: безопасное перепрограммирование pll_sys + VREG.  
// Порт из src/vfo/soft_tx.cpp (RU0AOG). Глобалы убраны: xosc_hz и  
// pll_nominal — параметры.  
// ============================================================================  
#pragma once  
#include <Arduino.h>  
  
typedef struct {  
  uint32_t fbdiv;       // множитель VCO (16..320)  
  uint32_t p1, p2;      // post-делители (p1 >= 2!)  
  uint32_t clk_sys_hz;  // итоговая частота шины, Гц  
  uint32_t vsel;        // VREG_VOLTAGE_*  
  bool     is_oc;  
  uint32_t refdiv;      // делитель опоры (>=1)  
} quad_vfo_pll_t;  
  
uint32_t quad_vfo_vsel_for(uint64_t clk_hz);  
  
// Перепрограммирование pll_sys: detach clk_peri -> XOSC -> pll_init ->  
// LOCK-таймаут -> SSI BAUDR -> clk_sys -> верификация x3 -> откат на  
// nominal. false — частота не подтверждена (шина на номинале).  
// NB: pll_init() pico-sdk ждёт LOCK БЕСКОНЕЧНО — внешний таймаут  
// защищает только наш дополнительный цикл, а не зависание внутри SDK.  
bool quad_vfo_set_clk_sys(const quad_vfo_pll_t *cfg, uint32_t vsel,  
                          const quad_vfo_pll_t *nominal, uint64_t xosc_hz);  
  
// Измеренная частота шины после последней успешной прошивки.  
uint32_t quad_vfo_clk_sys_hz(void);