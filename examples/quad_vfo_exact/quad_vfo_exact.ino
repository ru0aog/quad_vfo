// quad_vfo_exact.ino  
// ============================================================================  
// Два режима бок о бок:  
//   init_simple -> LO = freq (ошибка минимальна, в пределах шага Q8)  
//   полный init с if_shift -> LO = freq - ПЧ (режим приёмника)  
// ============================================================================  
#include <quad_vfo.h>  
  
void setup() {  
  Serial.begin(115200);  
  delay(2000);  
  
  // Режим 1: точная частота  
  if (quad_vfo_init_simple(pio1, 0)) {  
    quad_vfo_start(10000000);  
    Serial.printf("[exact]  req=10000000 -> LO=%lu Гц\n",  
                  (unsigned long)quad_vfo_actual_hz());  
    quad_vfo_stop();  
  }  
  
  // Режим 2: приёмник с ПЧ — тот же pio1, re-init транзакционен:  
  // старая SM освобождается только после успешной валидации новой.  
  quad_vfo_cfg_t cfg;  
  quad_vfo_default_cfg(&cfg);  
  cfg.pio         = pio1;  
  cfg.pin_i       = 0;  
  cfg.sm          = -1;  
  cfg.if_shift_hz = 11000u;   // nco-зоны уже заполнены default_cfg  
  
  if (quad_vfo_init(&cfg)) {  
    quad_vfo_start(10000000);  
    Serial.printf("[rx]     req=10000000 -> LO=%lu Гц (ожидалось 9989000)\n",  
                  (unsigned long)quad_vfo_actual_hz());  
  }  
}  
  
void loop() {}
