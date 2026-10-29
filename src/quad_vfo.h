// quad_vfo.h  
// ============================================================================  
// Автономная библиотека квадратурного гетеродина (quad VFO) для RP2040.  
// 4-тактовая PIO-программа выдаёт на 2 соседних пина два меандра одной  
// частоты со сдвигом 90° для SDR-смесителя.  
//  
// Последовательность set pins (bit0 -> pin_i = I, bit1 -> pin_i+1 = Q):  
//   0b01: I=1 Q=0   0b11: I=1 Q=1   0b10: I=0 Q=1   0b00: I=0 Q=0  
//   I: 1,1,0,0   Q: 0,1,1,0  -> Q ОТСТАЁТ от I на 90°.  
//   Если приёмнику нужен другой знак квадратуры — поменяйте пины I/Q  
//   местами или перестройте порядок фаз в pio_quad_instructions.  
//  
// Порт soft_rx.cpp из RP2040_radiomodem (RU0AOG).  
// Все зависимости от менеджера vfo/pins/soft_tx убраны: PIO, SM, пины,  
// PLL-лимиты и ПЧ — параметры quad_vfo_cfg_t.  
// ============================================================================  
#pragma once  
#include <Arduino.h>  
#include "hardware/pio.h"  
#include "quad_vfo_pll.h"  
  
typedef struct {  
  // --- Железо ---  
  PIO     pio;                 // pio0 / pio1  
  int     sm;                  // 0..3 или -1 -> pio_claim_unused_sm()  
  uint8_t pin_i;               // I = pin_i, Q = pin_i + 1 (обязательно смежные)  
  
  // --- PLL-подсистема ---  
  quad_vfo_pll_t pll_nominal;  // дефолтная шина + откат при провале lock  
  quad_vfo_pll_t pll_ceiling;  // потолок, если clk_sys_max_hz "не доказан"  
  uint64_t xosc_hz;            // опора PLL (обычно 12000000)  
  uint64_t clk_sys_min_hz;     // минимум окна сканера  
  uint64_t clk_sys_max_hz;     // паспортный потолок clk_sys  
  uint64_t pll_vco_max_hz;     // потолок VCO (cap 1600 МГц по datasheet)  
  uint64_t clk_sys_nominal_hz; // граница is_oc (133 МГц)  
  
  // --- ПЧ и NCO-зоны сканера ---  
  uint32_t if_shift_hz;        // LO = freq - if_shift  
  uint32_t nco_max_hz;         // макс |f_nco|  
  uint32_t nco_sweet_lo, nco_sweet_hi;    // "сладкая" зона  
  uint32_t nco_forbid_lo, nco_forbid_hi;  // запретная зона  
} quad_vfo_cfg_t;  
  
#ifdef __cplusplus  
extern "C" {  
#endif  
  
// Дефолты исходника: pio1, sm=-1, pin_i=0, XOSC=12 МГц, окно 240..420 МГц,  
// VCO <= 1600 МГц, ПЧ=11000, NCO-зоны 9600..14500 / 0..5000 / max 62500.  
void quad_vfo_default_cfg(quad_vfo_cfg_t *cfg);  
  
// Полный init. Валидация cfg, загрузка PIO-программы (в каждый используемый  
// PIO-блок отдельно), резервирование SM. Повторный init сначала  
// освобождает прежнюю SM и останавливает генерацию.  
bool quad_vfo_init(const quad_vfo_cfg_t *cfg);  
  
// Удобный init: дефолт-cfg + заданные pio/pin_i, SM — автовыбор.  
bool quad_vfo_init_simple(PIO pio, uint8_t pin_i);  
  
// Запуск LO на freq_hz (LO = freq_hz - cfg.if_shift_hz).  
// false, если: freq_hz <= if_shift_hz; скан дал 0 кандидатов;  
// переключение clk_sys провалилось и на номинале нарушены NCO-ограничения.  
bool quad_vfo_start(uint32_t freq_hz);  
  
// Остановка: SM off, пины в input, actual_hz = 0.  
void quad_vfo_stop(void);  
  
// Фактическая частота LO, Гц (0 — не запущен). Усечённая оценка по  
// конфигурации тактирования, не измерение.  
uint32_t quad_vfo_actual_hz(void);  
  
// Ошибка настройки LO в миллигерцах: (lo_цель - lo_факт) * 1000.  
// Знак: >0 — LO ниже целевой.  
int32_t quad_vfo_error_mhz(void);  
  
bool quad_vfo_active(void);  
  
// Диагностика в Serial.  
void quad_vfo_debug(void);  
  
#ifdef __cplusplus  
}  
#endif