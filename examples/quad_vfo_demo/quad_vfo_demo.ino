// quad_vfo_demo.ino — минимальный запуск квадратурного LO  
#include <quad_vfo.h>  
  
void setup() {  
  Serial.begin(115200);  
  delay(2000);  
  
  // Удобный уровень: pio1, I=GP0, Q=GP1, SM — автовыбор,  
  // все PLL/NCO-параметры — дефолты исходника.  
  if (!quad_vfo_init_simple(pio1, 0)) {  
    Serial.println("init failed");  
    while (1) delay(100);  
  }  
  
  // LO = 7100000 - if_shift(11000) = 7089000 Гц  
  if (!quad_vfo_start(7100000)) {  
    Serial.println("start failed — сканер не нашёл кандидата");  
  }  
  quad_vfo_debug();  
}  
  
void loop() {  
  if (Serial.available()) {  
    char c = (char)Serial.read();  
    if (c == 'd') quad_vfo_debug();  
    if (c == 's') { quad_vfo_stop();  Serial.println("stopped"); }  
    if (c == 'r') { quad_vfo_start(7100000); }  
    if (c == 'f') {  
      // "f" + число: f14074000\n  
      uint32_t hz = (uint32_t)Serial.parseInt();  
      if (hz > 0) quad_vfo_start(hz);  
    }  
  }  
}
