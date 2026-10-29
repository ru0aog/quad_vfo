// quad_vfo_pll.h  
// ============================================================================  
// PLL-подсистема quad_vfo: безопасное перепрограммирование pll_sys + VREG.  
// Порт из src/vfo/soft_tx.cpp (RU0AOG). Глобалы убраны: xosc_hz и  
// pll_nominal — параметры.  
//  
// ВАЖНО про LOCK: штатный pll_init() pico-sdk ждёт LOCK БЕСКОНЕЧНО  
// (tight_loop_contents). Внешние таймауты в этой библиотеке защищают  
// только ожидания ПОСЛЕ возврата pll_init — от зависания внутри SDK  
// они не защищают. Если требуется ограниченное время операции, соберите  
// с -DQUAD_VFO_BOUNDED_PLL: тогда регистры PLL настраиваются напрямую  
// с контролируемым ожиданием захвата (без pll_init).  
// ============================================================================  
#pragma once  
#include <Arduino.h>  
  
typedef struct {  
  uint32_t fbdiv;       // множитель VCO (16..320)  
  uint32_t p1, p2;      // post-делители (p1 >= 2!)  
  uint32_t clk_sys_hz;  // итоговая частота шины, Гц. Обязана быть  
                        // согласована с fbdiv/refdiv/p1/p2 и xosc —  
                        // проверяется в quad_vfo_init.  
  uint32_t vsel;        // VREG_VOLTAGE_*  
  bool     is_oc;  
  uint32_t refdiv;      // делитель опоры (>=1)  
} quad_vfo_pll_t;  
  
uint32_t quad_vfo_vsel_for(uint64_t clk_hz);  
  
// Перепрограммирование pll_sys: detach clk_peri -> XOSC -> PLL ->  
// LOCK -> SSI BAUDR -> clk_sys -> верификация frequency_count_khz x3.  
// false — целевая частота НЕ подтверждена. Тогда библиотека пытается  
// вернуть шину на nominal; если и это не удалось (LOCK не захвачен) —  
// шина остаётся на XOSC и quad_vfo_clk_sys_hz() возвращает xosc_hz.  
bool quad_vfo_set_clk_sys(const quad_vfo_pll_t *cfg, uint32_t vsel,  
                          const quad_vfo_pll_t *nominal, uint64_t xosc_hz);  
  
// Частота шины по последней подтверждённой прошивке (или xosc после  
// неудачного отката — система при этом работает на опоре).  
uint32_t quad_vfo_clk_sys_hz(void);