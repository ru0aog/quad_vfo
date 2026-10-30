// quad_vfo_demo.ino
// ============================================================================
// Базовый пример: квадратурный LO на pio1, I = GP0, Q = GP1.
// Режим init_simple: if_shift = 0, NCO-зоны выключены ->  
// quad_vfo_start(freq) ставит LO ТОЧНО на freq
//  
// Команды Serial (115200):
//   d         — диагностика (clk_sys, clkdiv, состояние SM)
//   s         — стоп
//   r         — перезапуск на последней частоте
//   f<Гц>     — установить частоту, напр. f14074000
// ============================================================================
#include <quad_vfo.h>

// целевая частота генератора
static uint32_t s_freq = 7100000;

// вывод отчёта
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

  // ожидание коннекта - до 5 сек
  uint32_t timeout = millis();
  while (!Serial && (millis() - timeout < 5000)) {  }

  // вывод приветствия
  Serial.println("\n ==== КВАДРАТУРНЫЙ ГЕНЕРАТОР ==== ");
 
  // инициализация генератора:
  // блок ввода-вывода PIO1, пины I = GP0, Q = GP1, SM — первая свободная
  if (!quad_vfo_init_simple(pio1, 0)) {
    Serial.println("quad_vfo_init_simple failed (заняты SM или нет памяти программ PIO)");
    while (1) delay(1000);
  }
  
  // настроить и включить генератор:
  // целевая частота - s_freq
  if (!quad_vfo_start(s_freq)) {
    Serial.println("start failed — сканер не нашёл делитель в допустимой ошибке");
  }

  // вывести отчёт
  report();
}

void loop() {
  // циклический опрос порта
  if (!Serial.available()) return;
  char c = (char)Serial.read();

  // игнорируем символы перевода строки и возврата каретки, 
  if (c == '\n' || c == '\r') return;

  // ВЫВОД ЭХО: Выводим всю принятую строку один раз в терминал
  Serial.print("\n> ");
  Serial.println(c);

  // выполнение команд
  if (c == 'd') quad_vfo_debug();
  else if (c == 's') { quad_vfo_stop();  Serial.println("stopped"); }
  else if (c == 'r') { quad_vfo_start(s_freq); report(); }
  else if (c == 'f') {
    uint32_t hz = (uint32_t)Serial.parseInt();
    if (hz > 0) {
      s_freq = hz;
      if (quad_vfo_start(hz)) report();
      else Serial.println("start failed — частота вне достижимого окна");
    }
  }
  else if (c != 's') { 
    Serial.println(" ==== СПРАВКА ПО КОМАНДАМ ==== ");
    Serial.println("d - вывод диагностических данных");
    Serial.println("r - перезапуск на последней частоте и вывод отчёта");
    Serial.println("f3650000 - перестройка генератора на частоту 3650000 Гц");
    Serial.println("s - остановка генератора");
  }
}
