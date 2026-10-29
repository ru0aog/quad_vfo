// quad_vfo_demo_cfg.ino  
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
  delay(2000);  
  
  quad_vfo_cfg_t cfg;  
  quad_vfo_default_cfg(&cfg);            // сначала дефолты...  
  
  cfg.pio            = pio0;             // ...потом правим  
  cfg.sm             = 2;                // явная SM (-1 = автовыбор)  
  cfg.pin_i          = 4;                // I = GP4, Q = GP5  
  cfg.clk_sys_max_hz = 380000000ULL;     // паспорт стабильности (OCTEST)  
  cfg.pll_vco_max_hz = 1600000000ULL;    // аппаратный потолок VCO  
  
  // --- Режим приёмника: LO ниже частоты приёма на величину ПЧ ---  
  // Зоны NCO идут из default_cfg (sweet 9600..14500, forbid 0..5000,  
  // max 62500) — обязаны быть заданы, если if_shift_hz != 0.  
  cfg.if_shift_hz    = 11000u;           // LO = freq - 11000  
  
  if (!quad_vfo_init(&cfg)) {  
    Serial.println("quad_vfo_init failed: проверьте валидность cfg "  
                   "(профили PLL, зоны NCO, занятость SM2 на pio0)");  
    while (1) delay(100);  
  }  
  
  // Приём 14074000 Гц -> LO = 14063000 Гц  
  if (!quad_vfo_start(14074000)) {  
    Serial.println("start failed — сканер не нашёл кандидата "  
                   "или делитель вне NCO-ограничений");  
  }  
  quad_vfo_debug();  
}  
  
void loop() {}
