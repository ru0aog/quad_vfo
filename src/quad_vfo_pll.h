// quad_vfo_pll.h
// ============================================================================
// PLL-подсистема quad_vfo: безопасное перепрограммирование pll_sys + VREG.
// Порт из src/vfo/soft_tx.cpp (RU0AOG). Глобалы убраны: xosc_hz и
// pll_nominal — параметры.
//
// Про LOCK: штатный pll_init() pico-sdk ждёт LOCK БЕСКОНЕЧНО и поэтому
// здесь НЕ используется. Регистры настраиваются напрямую по
// последовательности pll_deinit()+pll_init() из SDK (clocks.c) с
// ограниченным ожиданием захвата — вечного цикла нет ни в одном пути.
// Перед использованием сверьте pll_setup_bounded() с clocks.c вашей
// версии pico-sdk.
// ============================================================================
#pragma once
#include <Arduino.h>

typedef struct {
  uint32_t fbdiv;       // множитель VCO (16..320)
  uint32_t p1, p2;      // post-делители (p1 >= 2: p1=1 делит пополам
                        // недокументированно — отклоняется)
  uint32_t clk_sys_hz;  // итоговая частота шины, Гц — должна быть
                        // согласована с fbdiv/refdiv/p1/p2/xosc
  uint32_t vsel;        // VREG_VOLTAGE_*
  bool     is_oc;
  uint32_t refdiv;      // делитель опоры (>=1)
} quad_vfo_pll_t;

// VSEL по частоте шины (таблица исходника).
uint32_t quad_vfo_vsel_for(uint64_t clk_hz);

// Перепрограммирование pll_sys.
// true: целевая частота подтверждена.
// false + *out_nominal_ok=true: цель не подтверждена, но номинальный PLL
// восстановлен и clk_sys переключён на него.
// false + *out_nominal_ok=false: номинальный PLL тоже не подтверждён;
// clk_sys остаётся на XOSC, quad_vfo_clk_sys_hz() возвращает xosc_hz.
// out_nominal_ok может быть NULL.
bool quad_vfo_set_clk_sys(const quad_vfo_pll_t *cfg, uint32_t vsel,
                          const quad_vfo_pll_t *nominal,
                          uint64_t xosc_hz,
                          bool *out_nominal_ok);

// Частота шины по последней подтверждённой настройке (или XOSC
// после неудачного отката).
uint32_t quad_vfo_clk_sys_hz(void);  
  
// Измерение clk_sys аппаратным счётчиком FC0 (возвращает Гц,  
// квантование ~1 кГц). Для debug-вывода.  
uint32_t quad_vfo_clk_sys_meas_hz(void);

void quad_vfo_pll_set_quiet(bool on);
