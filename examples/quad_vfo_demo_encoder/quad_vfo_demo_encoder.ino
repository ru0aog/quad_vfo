// quad_vfo_demo_encoder.ino  
// ============================================================================  
// Расширенный пример + энкодер (порт из RP2040_beacon, vfo_hardware.cpp).  
//  
//   - явный PIO и SM, свои пины I/Q (смежные!), паспортные лимиты;  
//   - режим приёмника с ПЧ: LO = freq - if_shift_hz;  
//   - энкодер крутит частоту, ускорение от скорости вращения;  
//   - кнопка: 1-е нажатие — вкл/шаг 100 Гц, 2-е — шаг 1 кГц,  
//             3-е — шаг 10 кГц, 4-е — выкл;  
//   - при перестройке энкодером логи [QUAD]/[PLL] глушатся  
//     (quad_vfo_set_quiet).  
//  
// Пины (1-в-1 разводка маяка):  
//   ENC_A = GP20, ENC_B = GP21, ENC_BTN = GP19 (INPUT_PULLUP, на GND).  
//   I = GP28, Q = GP29 — см. примечание по pin_i ниже.  
// ============================================================================  
#include <quad_vfo.h>  
#include <hardware/gpio.h>  
#include <hardware/sync.h>  
#include <hardware/timer.h>  
  
// ==== Пины энкодера (как в маяке) ====  
#define PIN_ENC_A        20   // ENC_PIN_A маяка  
#define PIN_ENC_B        21   // ENC_PIN_B маяка  
#define PIN_ENC_BTN      19   // VFO_ENC_SW_PIN маяка  
  
#define ENC_TICKS_PER_DETENT  4     // тиков фазы на один щелчок  
#define ENC_DEBOUNCE_MS       30    // дебаунс кнопки, мс  
#define ENC_EDGE_US           1500  // антидребезг фронтов AB, мкс  
  
// ==== Состояние энкодера ====  
// ISR только накапливает "сырые" тики; детенты и ускорение — в enc_poll().  
static volatile int32_t  enc_pending_steps = 0;  
static volatile uint32_t g_enc_edge_us     = 0;  
static uint32_t          s_last_detent_us  = 0;  
  
// LUT переходов фазы (1-в-1 из маяка).  
static const int8_t enc_steps[16] = {  
   0, -1, +1,  0,  
  +1,  0,  0, -1,  
  -1,  0,  0, +1,  
   0, +1, -1,  0,  
};  
  
// ==== Частота / шаг / генератор ====  
static uint32_t g_freq_hz     = 14063000u;  
static uint32_t g_enc_step_hz = 100u;     // 100 / 1000 / 10000  
static uint8_t  g_step_idx    = 0;        // 0=100 Гц, 1=1 кГц, 2=10 кГц  
static bool     g_tx_on       = false;  
  
// ==== Обратный вывод ====  
static void report(void) {  
  Serial.printf("active=%d  LO=%lu Hz  err=%ld Hz  clk_sys=%lu Hz\n",  
                (int)quad_vfo_active(),  
                (unsigned long)quad_vfo_actual_hz(),  
                (long)quad_vfo_error_hz(),  
                (unsigned long)quad_vfo_clk_sys_hz());  
  Serial.printf("PLL: refdiv=%u fbdiv=%u postdiv=%u/%u  "  
                "VCO=%llu Hz %s\n",  
                (unsigned)quad_vfo_pll_refdiv(),  
                (unsigned)quad_vfo_pll_fbdiv(),  
                (unsigned)quad_vfo_pll_p1(),  
                (unsigned)quad_vfo_pll_p2(),  
                (unsigned long long)quad_vfo_pll_vco_hz(),  
                quad_vfo_pll_nominal() ? "(nominal)" : "(target)");  
}  
  
// ==== Перестройка в тихом режиме ====  
static void retune(uint32_t hz) {  
  quad_vfo_set_quiet(true);  
  quad_vfo_stop();  
  quad_vfo_start(hz);  
  quad_vfo_set_quiet(false);  
}  
  
// ==== ISR энкодера: только тики, никакой логики ====  
static void enc_gpio_isr(uint gpio, uint32_t events) {  
  (void)gpio; (void)events;  
  static uint8_t old_ab = 3;              // pull-up: покой = 11  
  uint32_t now = time_us_32();  
  if (now - g_enc_edge_us < ENC_EDGE_US) return;  // дребезг <1.5 мс  
  g_enc_edge_us = now;  
  
  uint8_t ab = ((uint8_t)(gpio_get(PIN_ENC_A) ? 1 : 0) << 1)  
             |  (uint8_t)(gpio_get(PIN_ENC_B) ? 1 : 0);  
  if (ab == old_ab) return;  
  int8_t d = enc_steps[(old_ab << 2) | ab];  
  old_ab = ab;  
  if (!d) return;  
  
  uint32_t ints = save_and_disable_interrupts();  
  enc_pending_steps += d;  
  restore_interrupts(ints);  
}  
  
// ==== Поллинг энкодера (как в маяке): детенты + ускорение ====  
// Возвращает количество детентов с учётом гейна скорости.  
// Неполный щелчок (<4 тиков) возвращается в накопитель.  
static int32_t enc_poll(void) {  
  uint32_t ints = save_and_disable_interrupts();  
  int32_t ticks = enc_pending_steps;  
  enc_pending_steps = 0;  
  restore_interrupts(ints);  
  if (!ticks) return 0;  
  
  int32_t detents = ticks / ENC_TICKS_PER_DETENT;  
  if (detents == 0) {  
    // Половина/четверть щелчка — вернуть тики, копим дальше.  
    ints = save_and_disable_interrupts();  
    enc_pending_steps += ticks;  
    restore_interrupts(ints);  
    return 0;  
  }  
  
  // Остаток — обратно в накопитель.  
  int32_t rem = ticks - detents * ENC_TICKS_PER_DETENT;  
  if (rem) {  
    ints = save_and_disable_interrupts();  
    enc_pending_steps += rem;  
    restore_interrupts(ints);  
  }  
  
  // Гейн от скорости вращения: чем меньше пауза между детентами,  
  // тем больше шаг.  
  uint32_t now = time_us_32();  
  uint32_t dt  = now - s_last_detent_us;  
  s_last_detent_us = now;  
  int32_t gain = (dt <  40000u) ? 10 :  
                 (dt < 150000u) ?  3 : 1;  
  
  return detents * gain;  
}  
  
// ==== Кнопка: опрос с дебаунсом (как в маяке, без IRQ) ====  
static bool enc_button_pressed(void) {  
  static bool     sw_last = true;         // INPUT_PULLUP: отпущена = 1  
  static uint32_t sw_ms   = 0;  
  bool     sw  = gpio_get(PIN_ENC_BTN);  
  uint32_t now = millis();  
  
  if (sw != sw_last && (now - sw_ms) > ENC_DEBOUNCE_MS) {  
    sw_ms   = now;  
    sw_last = sw;  
    if (sw == 0) return true;             // нажатие = замыкание на GND  
  }  
  return false;  
}  
  
// ==== Setup ====  
void setup() {  
  Serial.begin(115200);  
  
  uint32_t timeout = millis();  
  while (!Serial && (millis() - timeout < 5000)) { }  
  
  quad_vfo_cfg_t cfg;  
  quad_vfo_default_cfg(&cfg);  
  
  cfg.pio            = pio0;             // PIO0 (pio0 или pio1)  
  cfg.sm             = 0;                // SM0 (-1 = автовыбор)  
  cfg.pin_i          = 28;               // I=GP28, Q=GP29 (смежные!)  
  cfg.xosc_hz        = 12000000u;  
  
  cfg.clk_sys_min_hz = 100000000u;  
  cfg.clk_sys_max_hz = 380000000u;       // разгонный потолок  
  cfg.pll_vco_max_hz = 4200000000u;      // пользовательский потолок VCO  
  
  // номинал и потолок — даташитные диапазоны  
  cfg.pll_nominal.clk_sys_hz = 133000000u;  
  cfg.pll_nominal.refdiv     = 1;  
  cfg.pll_nominal.fbdiv      = 133;  
  cfg.pll_nominal.p1         = 6;  
  cfg.pll_nominal.p2         = 2;  
  
  cfg.pll_ceiling.clk_sys_hz = 380000000u;  
  cfg.pll_ceiling.refdiv     = 1;  
  cfg.pll_ceiling.fbdiv      = 95;  
  cfg.pll_ceiling.p1         = 3;  
  cfg.pll_ceiling.p2         = 1;  
  
  // режим приёмника: LO = freq − ПЧ; зоны NCO из default_cfg  
  cfg.if_shift_hz    = 11000u;  
  
  if (!quad_vfo_init(&cfg)) {  
    Serial.println("quad_vfo_init failed: проверьте валидность cfg "  
                   "(профили PLL, зоны NCO, занятость SM на pio0)");  
    while (1) delay(100);  
  }  
  
  if (!quad_vfo_start(g_freq_hz)) {  
    Serial.println("start failed — сканер не нашёл кандидата "  
                   "или делитель вне NCO-ограничений");  
    while (1) delay(100);  
  }  
  g_tx_on = true;  
  report();  
  quad_vfo_debug();  
  
  // ==== Энкодер: пины ====  
  gpio_init(PIN_ENC_A);   gpio_set_dir(PIN_ENC_A,   GPIO_IN);  
  gpio_init(PIN_ENC_B);   gpio_set_dir(PIN_ENC_B,   GPIO_IN);  
  gpio_init(PIN_ENC_BTN); gpio_set_dir(PIN_ENC_BTN, GPIO_IN);  
  gpio_pull_up(PIN_ENC_A);  
  gpio_pull_up(PIN_ENC_B);  
  gpio_pull_up(PIN_ENC_BTN);  
  
  // IRQ только на фазы A/B; кнопка опрашивается в loop().  
  gpio_set_irq_enabled_with_callback(PIN_ENC_A,  
      GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &enc_gpio_isr);  
  gpio_set_irq_enabled(PIN_ENC_B,  
      GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);  
  
  Serial.println("=== КВАДРАТУРНЫЙ ГЕНЕРАТОР + ЭНКОДЕР ===");  
  Serial.println("кнопка: вкл/100 Гц -> 1 кГц -> 10 кГц -> выкл");  
  Serial.printf("freq=%lu Hz  step=%lu Hz\n",  
                (unsigned long)g_freq_hz,  
                (unsigned long)g_enc_step_hz);  
}  
  
// ==== Loop ====  
void loop() {  
  // --- Вращение энкодера: детенты * гейн * шаг ---  
  int32_t d = enc_poll();  
  if (d) {  
    int64_t f = (int64_t)g_freq_hz + (int64_t)d * (int64_t)g_enc_step_hz;  
    if (f <  1000000LL) f =  1000000LL;  
    if (f > 45000000LL) f = 45000000LL;   // QUAD_VFO_LO_MAX_HZ  
    g_freq_hz = (uint32_t)f;  
  
    if (g_tx_on) {                        // перестройка только при вкл  
      retune(g_freq_hz);  
    }  
    Serial.printf("freq=%lu Hz  step=%lu Hz\n",  
                  (unsigned long)g_freq_hz,  
                  (unsigned long)g_enc_step_hz);  
  }  
  
  // --- Кнопка: цикл вкл/100 Гц -> 1 кГц -> 10 кГц -> выкл ---  
  if (enc_button_pressed()) {  
    if (!g_tx_on) {  
      // 1-е нажатие из off: включаем генератор, шаг 100 Гц  
      g_step_idx    = 0;  
      g_enc_step_hz = 100u;  
      if (quad_vfo_start(g_freq_hz)) {  
        g_tx_on = true;  
        Serial.println("[ENC] генератор ВКЛ, шаг 100 Гц");  
      } else {  
        Serial.println("[ENC] start failed — см. логи сканера");  
      }  
    } else if (g_step_idx < 2) {  
      // 2-е и 3-е нажатия: переключаем шаг  
      g_step_idx++;  
      g_enc_step_hz = (g_step_idx == 1) ? 1000u : 10000u;  
      Serial.printf("[ENC] шаг перестройки: %lu Гц\n",  
                    (unsigned long)g_enc_step_hz);  
    } else {  
      // 4-е нажатие: выключаем генератор  
      quad_vfo_stop();  
      g_tx_on    = false;  
      g_step_idx = 0;  
      Serial.println("[ENC] генератор ВЫКЛ");  
    }  
  }  
}
