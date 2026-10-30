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
  
  // ожидание коннекта — до 5 сек  
  uint32_t timeout = millis();  
  while (!Serial && (millis() - timeout < 5000)) { }  
  
  quad_vfo_cfg_t cfg;  
  quad_vfo_default_cfg(&cfg);  
  
  cfg.pio            = pio0;             // PIO0  (pio0 или pio1)
  cfg.sm             = 0;                // SM0 (0, 1, 2 или 3. -1 = автовыбор)  
  cfg.pin_i          = 0;                // I = GP0, Q = GP1 (Любая смежная пара
                                         // pin_i (I) и pin_i+1 (Q), ограничение: pin_i ≤ 28
  cfg.clk_sys_max_hz = 380000000ULL;     // потолок системной шины 380 МГц  
  cfg.pll_vco_max_hz = 4200000000ULL;    // потолок VCO 4.2 ГГц (выше даташита —  
                                         // но LOCK всё равно  подтверждается измерением)  
  
  // --- Режим приёмника: LO ниже частоты приёма на величину ПЧ ---  
  // Зоны NCO идут из default_cfg (sweet 9600..14500, forbid 0..5000,  
  // max 62500) — обязаны быть заданы, если if_shift_hz != 0.  
  cfg.if_shift_hz    = 11000u;           // ПЧ 11 кГц: LO = freq - 11000  

  if (!quad_vfo_init(&cfg)) {  
    Serial.println("quad_vfo_init failed: проверьте cfg "  
                   "(профили PLL, зоны NCO, занятость SM0 на pio0)");  
    while (1) delay(100);  
  }  

  uint32_t freq = 3650250;
  
  // LO = 14063000 Гц + 11000 → приём на 14074000 Гц 
  if (!quad_vfo_start(freq)) {  
    Serial.println("start failed — сканер не нашёл кандидата "  
                   "или делитель вне NCO-ограничений");  
    while (1) delay(100);  
  }  
  
  // --- Обратный вывод: что реально получилось ---  
  Serial.printf("LO target  = %lu Hz\n", (unsigned long)(freq - cfg.if_shift_hz));  
  Serial.printf("LO actual  = %lu Hz\n", (unsigned long)quad_vfo_actual_hz());  
  Serial.printf("LO error   = %ld Hz\n", (long)quad_vfo_error_hz());  
  Serial.printf("clk_sys    = %lu Hz\n", (unsigned long)quad_vfo_clk_sys_hz());  
  Serial.printf("active     = %d\n",     (int)quad_vfo_active());  
  
  // Полный дамп: clk_sys (cached/measured), PIO/SM/clkdiv,  
  // PLL refdiv/fbdiv/postdiv/VCO, LO и ошибка.  
  quad_vfo_debug();  
}  
  
void loop() {}
