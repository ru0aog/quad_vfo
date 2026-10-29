// quad_vfo_demo_cfg.ino — полная конфигурация  
#include <quad_vfo.h>  
  
void setup() {  
  Serial.begin(115200);  
  delay(2000);  
  
  quad_vfo_cfg_t cfg;  
  quad_vfo_default_cfg(&cfg);          // сначала дефолты...  
  
  cfg.pio              = pio0;         // ...потом правим  
  cfg.sm               = 2;            // явная SM (или -1 = авто)  
  cfg.pin_i            = 4;            // I=GP4, Q=GP5  
  cfg.clk_sys_max_hz   = 380000000ULL; // паспорт стабильности после OCTEST  
  cfg.pll_vco_max_hz   = 3000000000ULL;  
  cfg.if_shift_hz      = 11000u;  
  // cfg.pll_ceiling при необходимости тоже правится  
  
  if (!quad_vfo_init(&cfg)) {  
    Serial.println("init failed");  
    while (1) delay(100);  
  }  
  
  quad_vfo_start(14074000);            // FT8 20м  
  quad_vfo_debug();  
}  
  
void loop() {}
