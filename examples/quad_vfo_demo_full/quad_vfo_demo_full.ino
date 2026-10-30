// quad_vfo_demo_full.ino
// ============================================================================  
// Расширенный пример: полная конфигурация quad_vfo_cfg_t.  
//   - явный PIO и SM;  
//   - свои пины I/Q (смежные!);  
//   - паспортные лимиты clk_sys / VCO;  
//   - режим приёмника с ПЧ (LO = freq - if_shift_hz), как в soft_rx.  
// ============================================================================
#include <quad_vfo.h>

void setup() {  
  Serial.begin(115200);  

  // ожидание коннекта - до 5 сек
  uint32_t timeout = millis();
  while (!Serial && (millis() - timeout < 5000)) {  } 
  
  // объявляем структуры
  quad_vfo_cfg_t cfg;  
  quad_vfo_default_cfg(&cfg);
  
  cfg.pio            = pio0;             // назначаем PIO = PIO0
  cfg.sm             = 2;                // назначаем SM2 (-1 = автовыбор)
  cfg.pin_i          = 4;                // назначаем пару пинов I = GP4, Q = GP5  
  cfg.clk_sys_max_hz = 380000000ULL;     // наибольшая частота системной шины 380 МГц
  cfg.pll_vco_max_hz = 1600000000ULL;    // наибольшая частота VCO 1600 МГц
  
  // --- Режим приёмника: LO ниже частоты приёма на величину ПЧ ---  
  // Зоны NCO идут из default_cfg (sweet 9600..14500, forbid 0..5000,  
  // max 62500) — обязаны быть заданы, если if_shift_hz != 0.  
  cfg.if_shift_hz    = 11000u;           // частота ПЧ 11 кГц, LO = freq - 11000  
  
  // проверка валидности настроек
  if (!quad_vfo_init(&cfg)) {
    Serial.println("quad_vfo_init failed: проверьте валидность cfg "  
                   "(профили PLL, зоны NCO, занятость SM2 на pio0)");  
    while (1) delay(100);  
  }  
  
  // запуск генератора на частоте 14063000 (LO = 14063000  + 11000 ПЧ = приём на 14074000 Гц)
  if (!quad_vfo_start(14074000)) {  
    Serial.println("start failed — сканер не нашёл кандидата "  
                   "или делитель вне NCO-ограничений");  
  }  
  quad_vfo_debug();  
}  
  
void loop() {}
