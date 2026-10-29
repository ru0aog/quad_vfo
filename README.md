# quad_vfo  
  
Автономная библиотека квадратурного гетеродина (quadrature LO / VFO) для  
Raspberry Pi Pico / RP2040. Генерирует на двух соседних GPIO два меандра  
одной частоты со сдвигом фазы 90° (I/Q) — для питания SDR-смесителей  
(Tayloe, QSD/QSE, фазовых детекторов).  

  
## Возможности  
  
- Квадратурный выход I/Q 90° на любой паре соседних GPIO через PIO  
  (4-тактовая программа, последовательность 01→11→10→00).  
- Свободный выбор PIO-блока (pio0/pio1) и state machine — явно или  
  автовыбор через `pio_claim_unused_sm()`.  
- Полноценный сканер PLL: перепрошивает `clk_sys` так, чтобы делитель  
  PIO был **целым** — чистый LO без дробного джиттера. Дробный делитель  
  используется только как компромисс с явным штрафом.  
- Критерии выбора конфигурации (как в исходнике):  
  - **П1** — жёсткий запрет «дырки в звуке» (NCO в запретной зоне);  
  - **П2** — предпочтение «сладкой зоны» NCO;  
  - **П3** — штраф за дробный делитель;  
  - **П4** — штраф за дробь вблизи k/8;  
  - **П5** — предпочтение шины ближе к потолку;  
  - **П6** — fallback «наименьшее зло».  
- Безопасное перепрограммирование `pll_sys` + `vreg`: отход на XOSC,  
  LOCK-таймаут, корректировка SSI BAUDR флэша, самопроверка  
  `frequency_count_khz` с 3 ретраями, откат на номинал при провале.  
- Все лимиты (окно clk_sys, потолок VCO, ПЧ, NCO-зоны) параметризованы  
  через `quad_vfo_cfg_t` — библиотека не содержит глобальных «магических»  
  зависимостей.  
  
## Установка  

Скопируйте каталог `quad_vfo/` в `libraries/` вашего Arduino-окружения (ядро [arduino-pico](https://github.com/earlephilhower/arduino-pico))
или в `lib/` проекта PlatformIO.

## Быстрый старт  
  
```cpp  
#include <quad_vfo.h>  
  
void setup() {  
  Serial.begin(115200);  
  
  // pio1, I=GP0, Q=GP1, SM — автовыбор, лимиты — дефолтные  
  quad_vfo_init_simple(pio1, 0);  
  
  // LO = 7100000 - if_shift_hz(11000) = 7089000 Гц  
  quad_vfo_start(7100000);  
}  
  
void loop() {}
```

📂 Структура библиотеки

text
libraries/quad_vfo/
├── library.properties
├── src/
│   ├── quad_vfo.h — публичный API и quad_vfo_cfg_t
│   ├── quad_vfo.cpp — PIO-программа, сканер, start/stop/debug
│   ├── quad_vfo_pll.h — quad_vfo_pll_t, set_clk_sys, vsel_for
│   └── quad_vfo_pll.cpp — безопасная перепрошивка pll_sys + vreg
└── examples/
    ├── quad_vfo_demo/ — минимальный запуск (pio1, GP0/GP1)
    └── quad_vfo_demo_cfg/ — полная конфигурация (свой PIO/SM/лимиты)
Используйте код с осторожностью.
Требования: ядро arduino-pico (Pico SDK: hardware/pio, hardware/clocks, hardware/pll, hardware/vreg, hardware/sync).

⚙️ API


Основные функции


Функция	Назначение
quad_vfo_default_cfg(&cfg)	Заполнить quad_vfo_cfg_t дефолтами исходника
quad_vfo_init(&cfg)	Полный init: валидация, загрузка PIO-программы, резервирование SM
quad_vfo_init_simple(pio, pin_i)	Удобный init: дефолт-cfg, SM — автовыбор
quad_vfo_start(freq_hz)	Сканер PLL + запуск LO. false — пустой скан, LO не тронут
quad_vfo_stop()	SM off, оба пина в input
quad_vfo_actual_hz()	Фактическая частота LO (0 — не запущен)
quad_vfo_active()	true — генерация активна
quad_vfo_debug()	Диагностика в Serial: clk_sys, clkdiv, ctrl, lo_hz


Низкоуровневые PLL-функции

(Доступны, но обычно вызываются изнутри quad_vfo_start):

Функция	Назначение
quad_vfo_vsel_for(clk_hz)	VSEL по частоте шины
quad_vfo_set_clk_sys(&cfg, vsel, &nominal, xosc)	Перепрошивка pll_sys с верификацией
quad_vfo_clk_sys_hz()	Измеренная частота шины после последней прошивки


🛠️ Структура конфигурации quad_vfo_cfg_t

cpp
typedef struct {  
  // --- Железо ---  
  PIO     pio;                 // pio0 / pio1  
  int     sm;                  // 0..3 или -1 → pio_claim_unused_sm()  
  uint8_t pin_i;               // I = pin_i, Q = pin_i + 1 (смежные!)  
  
  // --- PLL-подсистема ---  
  quad_vfo_pll_t pll_nominal;  // дефолтная шина + откат при провале lock  
  quad_vfo_pll_t pll_ceiling;  // потолок, если clk_sys_max_hz "не доказан"  
  uint64_t xosc_hz;            // опора PLL (обычно 12000000)  
  uint64_t clk_sys_min_hz;     // минимум окна сканера  
  uint64_t clk_sys_max_hz;     // паспортный потолок clk_sys  
  uint64_t pll_vco_max_hz;     // потолок VCO  
  uint64_t clk_sys_nominal_hz; // граница is_oc  
  
  // --- ПЧ и NCO-зоны сканера ---  
  uint32_t if_shift_hz;        // LO = freq - if_shift  
  uint32_t nco_max_hz;         // макс |f_nco|  
  uint32_t nco_sweet_lo, nco_sweet_hi;    // "сладкая" зона  
  uint32_t nco_forbid_lo, nco_forbid_hi;  // запретная зона  
} quad_vfo_cfg_t;
Используйте код с осторожностью.

Дефолты quad_vfo_default_cfg():


Поле	Значение
pio / sm / pin_i	pio1 / −1 (авто) / 0 (I=GP0, Q=GP1)
pll_nominal	133 МГц (fbdiv=133, p1=6, p2=2, vsel=default)
pll_ceiling	~380 МГц @ VREG_1_30 (fbdiv=95, p1=3, p2=1)
xosc_hz	12 000 000
clk_sys_min_hz	240 000 000
clk_sys_max_hz	133 000 000 («не доказано» → используется pll_ceiling)
pll_vco_max_hz	1 600 000 000
clk_sys_nominal_hz	133 000 000
if_shift_hz	11 000
nco_max_hz	62 500
nco_sweet_*	9 600 … 14 500
nco_forbid_*	0 … 5 000


Пример полной конфигурации:

cpp
quad_vfo_cfg_t cfg;  
quad_vfo_default_cfg(&cfg);  
  
cfg.pio            = pio0;  
cfg.sm             = 2;                 // или -1 = автовыбор  
cfg.pin_i          = 4;                 // I=GP4, Q=GP5  
cfg.clk_sys_max_hz = 380000000ULL;      // если пройден OCTEST  
cfg.pll_vco_max_hz = 3000000000ULL;  
cfg.if_shift_hz    = 11000;  
  
quad_vfo_init(&cfg);  
quad_vfo_start(14074000);               // FT8 20м
Используйте код с осторожностью.

🔍 Как это работает

text
freq_hz ──(− if_shift_hz)──> lo_hz (целевая частота LO)  
                                │  
                сканер PLL: refdiv 1..3 × p1 2..7 × p2 1..7 × fbdiv 16..320  
                для каждой шины — целый и дробный делитель PIO (Q8)  
                                │  
              критерии П1..П6 ──> best (легальный) / fallback (наименьшее зло)  
                                │  
     quad_vfo_set_clk_sys: vreg → XOSC → pll_init → LOCK → SSI BAUDR  
                          → clk_sys → верификация ×3 → откат при провале  
                                │  
          clkdiv пересчитан от реальной clock_get_hz(clk_sys)  
                                │  
      SM config: wrap 0..3, set_pins(pin_i,2), pio_sm_init →  
      pindirs output (ПОСЛЕДНИМ) → pio_sm_restart → enable
Используйте код с осторожностью.

PIO-программа

(Статическая, 4 инструкции, занимает 4 слова инструкционной памяти выбранного PIO):
text
set pins, 0b01   ; Q=1 I=0  
set pins, 0b11   ; Q=1 I=1  
set pins, 0b10   ; Q=0 I=1  
set pins, 0b00   ; Q=0 I=0
Используйте код с осторожностью.
\(\text{LO}=\frac{\text{clk\_sys}}{4\times \text{div}}\)
где div — делитель SM в формате Q8.
Разность фаз I/Q = 90° при любом делителе.

⚠️ Важные предупреждения

• quad_vfo_start() перепрошивает системную шину clk_sys. Это глобальный эффект: меняются скорости UART, SPI, тайминги USB. Если в скетче есть другие потребители шины, координируйте переходы сами (в исходном проекте это делал trx_switch). Скорость Serial после смены шины может «поплыть» — перенастройте UART или используйте USB-Serial (CDC), который тактирован от pll_usb.
• Пины должны быть соседними (pin_i, pin_i+1) — требование set pins с count=2 в PIO. init проверяет pin_i+1 ≤ 29.
• Разгон. Сканер работает в окне 240–420 МГц и поднимает VREG до 1.30 В — за пределами спецификации RP2040 (номинал 133 МГц / 1.10 В). Потолок надо задавать по факту стабильности конкретного кристалла (в исходном проекте — тестом OCTEST). Заводское clk_sys_max_hz намеренно «не доказано» (133 МГц) → используется pll_ceiling 380 МГц. Поставьте своё значение, если 380 МГц не гарантированы.
• XOSC. quad_vfo_default_cfg() ставит 12 МГц. Если кварц другой — обязательно поправьте cfg.xosc_hz, иначе весь расчёт будет неверным.
• PLL pll_sys монополизируется на время прошивки. pll_usb не трогается.
• SM занимается навсегда после init (pio_sm_claim / pio_claim_unused_sm); API возврата SM нет — перезапустите init только после сброса PIO.
• Библиотека не потокобезопасна: quad_vfo_start нельзя вызывать из ISR или со второго ядра без внешней синхронизации.

🛠️ Диагностика

quad_vfo_debug() печатает в Serial:
text
[LODBG] clk_sys: cached=380011 kHz  measured=379980 kHz  
[LODBG] PIO=1 SM=0  clkdiv=0x0D2900 (int=3370 frac=0)  
[LODBG] SM ctrl=0x00000001 enabled=1  execctrl=0x0001F800  
[LODBG] lo_hz=14063000  active=1
Используйте код с осторожностью.
При ошибках сканер пишет [QUAD] ОШИБКА (пустой скан, LO не тронут) или [QUAD] ВНИМАНИЕ (fallback «наименьшее зло», |f_nco| > nco_max).
