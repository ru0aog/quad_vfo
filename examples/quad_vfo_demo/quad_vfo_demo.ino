// quad_vfo_demo.ino  
// ============================================================================  
// Базовый пример: квадратурный LO на pio1, I = GP0, Q = GP1.  
// Режим init_simple: if_shift = 0, NCO-зоны выключены ->  
// quad_vfo_start(freq) ставит LO ТОЧНО на freq (не freq - 11000!).  
//  
// Команды Serial (115200):  
//   d         — диагностика (clk_sys, clkdiv, состояние SM)  
//   s         — стоп  
//   r         — перезапуск на последней частоте  
//   f<Гц>     — установить частоту, напр. f14074000  
// ============================================================================  
#include <quad_vfo.h>  
  
static uint32_t s_freq = 7100000;  
  
static void report(void) {  
  Serial.printf("active=%d  LO=%lu Гц  расч.ошибка=%ld мГц  "  
                "clk_sys=%lu МГц\n",  
                (int)quad_vfo_active(),  
                (unsigned long)quad_vfo_actual_hz(),  
                (long)quad_vfo_error_mhz(),  
                (unsigned long)(quad_vfo_clk_sys_hz() / 1000000UL));  
}  
  
void setup() {  
  Serial.begin(115200);  
  delay(2000);  
  
  // pio1, I = GP0, Q = GP1, SM — автовыбор, остальное — дефолты.  
  if (!quad_vfo_init_simple(pio1, 0)) {  
    Serial.println("quad_vfo_init_simple failed "  
                   "(заняты SM или нет памяти программ PIO)");  
    while (1) delay(100);  
  }  
  
  // LO = 7100000 Гц ровно — ПЧ-сдвига в простом режиме нет.  
  if (!quad_vfo_start(s_freq)) {  
    Serial.println("start failed — сканер не нашёл делитель "  
                   "в допустимой ошибке");  
  }  
  report();  
}  
  
void loop() {  
  if (!Serial.available()) return;  
  char c = (char)Serial.read();  
  
  if (c == 'd') quad_vfo_debug();  
  else if (c == 's') { quad_vfo_stop();  Serial.println("stopped"); }  
  else if (c == 'r') { quad_vfo_start(s_freq); report(); }  
  else if (c == 'f') {  
    uint32_t hz = (uint32_t)Serial.parseInt();  
    if (hz > 0) {  
      s_freq = hz;  
      if (quad_vfo_start(hz)) report();  
      else Serial.println("start failed — частота вне "  
                          "достижимого окна");  
    }  
  }  
}
