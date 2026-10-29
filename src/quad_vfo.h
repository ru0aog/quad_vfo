// quad_vfo.h  
// ============================================================================  
// Автономная библиотека квадратурного гетеродина (quad VFO) для RP2040.  
// 4-тактовая PIO-программа выдаёт на 2 соседних пина последовательность  
// Q,I = 01→11→10→00 — два меандра f/1 со сдвигом 90° для SDR-смесителя.  
//  
// Порт soft_rx.cpp из RP2040_radiomodem (GPLv3, RU0AOG).  
// Зависимости от менеджера vfo/pins/soft_tx УДАЛЕНЫ: PIO, SM, пины,  
// PLL-лимиты и ПЧ — всё параметры quad_vfo_cfg_t.  
// ============================================================================  
#pragma once  
#include <Arduino.h>  
#include "hardware/pio.h"  
#include "quad_vfo_pll.h"  
  
// ---------------------------------------------------------------------------  
// Профиль PLL-шины {fbdiv, p1, p2, clk_sys_hz, vsel, is_oc, refdiv}  
// (тип объявлен в quad_vfo_pll.h как quad_vfo_pll_t)  
// ---------------------------------------------------------------------------  
  
typedef struct {  
  // --- Железо ---  
  PIO     pio;                 // pio0 / pio1 / pio2  
  int     sm;                  // 0..3, либо -1 → pio_claim_unused_sm()  
  uint8_t pin_i;               // I = pin_i, Q = pin_i + 1 (должны быть смежны)  
  
  // --- PLL-подсистема ---  
  quad_vfo_pll_t pll_nominal;  // дефолтная шина (fallback в сканере и откат)  
  quad_vfo_pll_t pll_ceiling;  // потолок, если clk_sys_max_hz "не доказан"  
  uint64_t xosc_hz;            // опора PLL (XOSC)  
  uint64_t clk_sys_min_hz;     // минимум окна сканера (240 МГц в исходнике)  
  uint64_t clk_sys_max_hz;     // паспортный потолок clk_sys  
  uint64_t pll_vco_max_hz;     // паспортный потолок VCO  
  uint64_t clk_sys_nominal_hz; // граница is_oc (133 МГц)  
  
  // --- ПЧ и NCO-зоны сканера ---  
  uint32_t if_shift_hz;        // LO = freq - if_shift (было RX_IF_SHIFT_HZ)  
  uint32_t nco_max_hz;         // макс |f_nco| (62500)  
  uint32_t nco_sweet_lo;       // сладкая зона NCO (9600)  
  uint32_t nco_sweet_hi;       //                      (14500)  
  uint32_t nco_forbid_lo;      // запретная "дырка в звуке" (0)  
  uint32_t nco_forbid_hi;      //                            (5000)  
} quad_vfo_cfg_t;  
  
#ifdef __cplusplus  
extern "C" {  
#endif  
  
// Заполнить cfg дефолтами исходника (pio1, sm=-1, pin_i=0, XOSC=12 МГц,  
// окно 240..420 МГц, VCO≤4.3 ГГц, ПЧ=11000, NCO-зоны как в soft_rx).  
void quad_vfo_default_cfg(quad_vfo_cfg_t *cfg);  
  
// Полный init: сохраняет cfg, загружает PIO-программу, резервирует SM.  
// Проверки: pin_i+1 ≤ 29; sm ∈ {-1,0..3}; явный sm не занят.  
// Пины в output НЕ переводит — направление ставится в start.  
bool quad_vfo_init(const quad_vfo_cfg_t *cfg);  
  
// Удобный init: дефолт-cfg + заданные pio/pin_i, sm = автовыбор.  
bool quad_vfo_init_simple(PIO pio, uint8_t pin_i);  
  
// Запуск LO на freq_hz (LO = freq_hz − cfg.if_shift_hz).  
// Полный сканер PLL П1..П6 + fallback "наименьшее зло" + перепрошивка  
// clk_sys через quad_vfo_set_clk_sys(). Возвращает false, если скан  
// не дал ни одного кандидата (LO не тронут).  
bool quad_vfo_start(uint32_t freq_hz);  
  
// Остановка: SM off, оба пина в input.  
void quad_vfo_stop(void);  
  
// Фактическая частота LO, Гц (0 — не запущен).  
uint32_t quad_vfo_actual_hz(void);  
  
// true — генерация активна.  
bool quad_vfo_active(void);  
  
// Диагностика в Serial: clk_sys cached/measured, clkdiv, ctrl, lo_hz.  
void quad_vfo_debug(void);  
  
#ifdef __cplusplus  
}  
#endif
