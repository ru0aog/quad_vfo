// quad_vfo_pll.cpp  
// Порт PLL-подсистемы из src/vfo/soft_tx.cpp (RU0AOG) + правки аудита.  
//  
// Настройка PLL — своя, без pll_init() SDK (тот ждёт LOCK бесконечно).  
// Последовательность повторяет pll_deinit()+pll_init() из clocks.c:  
//   PWR: PD|VCOPD|POSTDIVPD = 1     (PLL полностью выключен)  
//   CS:  refdiv (маскированно, только поле REFDIV — не трогаем LOCK/BYPASS)  
//   FBDIV_INT  
//   PWR: PD|VCOPD = 0               (включаем VCO и PLL)  
//   ждём LOCK с таймаутом QUAD_LOCK_TIMEOUT  
//   PRIM: POSTDIV1|POSTDIV2  
//   PWR: POSTDIVPD = 0              (пускаем выход)  
// Откат (rollback_nominal) управляет прерываниями САМ — может быть  
// вызван из любого контекста; при неподтверждённом LOCK clk_sys НЕ  
// переключается на PLL и остаётся на XOSC.  
#include "quad_vfo_pll.h"  
#include "hardware/clocks.h"  
#include "hardware/pll.h"  
#include "hardware/sync.h"  
#include "hardware/timer.h"  
#include "hardware/vreg.h"  
#include "hardware/structs/ssi.h"  
#include "hardware/structs/pll.h"  
#include "hardware/regs/pll.h"  
#include <stdio.h>  
  
#define QUAD_LOCK_TIMEOUT 20000u    // итераций ожидания LOCK  
  
static uint32_t s_clk_sys_hz = 133000000;  
  
uint32_t quad_vfo_clk_sys_hz(void) { return s_clk_sys_hz; }  
  
uint32_t quad_vfo_vsel_for(uint64_t clk_hz) {  
  if (clk_hz >= 250000000ULL)      return VREG_VOLTAGE_1_30;  
  else if (clk_hz >= 200000000ULL) return VREG_VOLTAGE_1_25;  
  else if (clk_hz > 133000000ULL)  return VREG_VOLTAGE_1_15;  
  return VREG_VOLTAGE_DEFAULT;  
}  
  
// ---------------------------------------------------------------------------  
static void __not_in_flash_func(detach_peri)(void) {  
  clock_configure(clk_peri, 0,  
                  CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,  
                  48u * 1000000u, 48u * 1000000u);  
}  
  
// clk_sys -> XOSC (SCK флэша безопасно). Вызывать при выключенных IRQ.  
static void __not_in_flash_func(clk_sys_to_xosc)(uint64_t xosc_hz) {  
  clock_configure(clk_sys,  
                  CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,  
                  CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_XOSC_CLKSRC,  
                  xosc_hz, xosc_hz);  
}  
  
// clk_sys -> PLL_SYS. Вызывать при выключенных IRQ.  
static void __not_in_flash_func(clk_sys_to_pll)(uint64_t hz) {  
  clock_configure(clk_sys,  
                  CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,  
                  CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,  
                  hz, hz);  
}  
  
// Контролируемое ожидание LOCK.  
static bool __not_in_flash_func(wait_pll_lock)(void) {  
  uint32_t timeout = QUAD_LOCK_TIMEOUT;  
  while (!(pll_sys_hw->cs & PLL_CS_LOCK_BITS) && --timeout) {}  
  return (pll_sys_hw->cs & PLL_CS_LOCK_BITS) != 0;  
}  
  
// Настройка pll_sys напрямую по регистрам — эквивалент  
// pll_deinit()+pll_init() SDK с ограниченным ожиданием LOCK.  
// Параметры — уже вычисленные refdiv/fbdiv/p1/p2 (не частоты).  
// false — LOCK не подтверждён; PLL оставлен выключенным (PD|VCOPD=1),  
// clk_sys вызывающий обязан держать на XOSC.  
static bool __not_in_flash_func(pll_setup_bounded)(uint32_t refdiv,  
                                                   uint32_t fbdiv,  
                                                   uint32_t p1,  
                                                   uint32_t p2) {  
  // 1. Полное выключение: PD + VCOPD + POSTDIVPD = 1 (как pll_deinit).  
  hw_set_bits(&pll_sys_hw->pwr,  
              PLL_PWR_PD_BITS | PLL_PWR_VCOPD_BITS | PLL_PWR_POSTDIVPD_BITS);  
  
  // 2. REFDIV — маскированной записью, LOCK и прочие поля cs не трогаем.  
  hw_write_masked(&pll_sys_hw->cs,  
                  refdiv << PLL_CS_REFDIV_LSB,  
                  PLL_CS_REFDIV_BITS);  
  
  // 3. Множитель VCO.  
  pll_sys_hw->fbdiv_int = fbdiv;  
  
  // 4. Включаем VCO и PLL (снимаем PD|VCOPD; POSTDIVPD пока остаётся —  
  // выход ещё не подключён, как в pll_init).  
  hw_clear_bits(&pll_sys_hw->pwr, PLL_PWR_PD_BITS | PLL_PWR_VCOPD_BITS);  
  
  // 5. Контролируемое ожидание захвата.  
  if (!wait_pll_lock()) {  
    // Нет LOCK — выключаем обратно и честно отказываемся.  
    hw_set_bits(&pll_sys_hw->pwr,  
                PLL_PWR_PD_BITS | PLL_PWR_VCOPD_BITS | PLL_PWR_POSTDIVPD_BITS);  
    return false;  
  }  
  
  // 6. Пост-делители и выпуск выхода.  
  pll_sys_hw->prim = (p1 << PLL_PRIM_POSTDIV1_LSB) |  
                     (p2 << PLL_PRIM_POSTDIV2_LSB);  
  hw_clear_bits(&pll_sys_hw->pwr, PLL_PWR_POSTDIVPD_BITS);  
  return true;  
}  
  
// Откат на номинальный профиль. Прерываниями управляет САМА.  
// true — номинал подтверждён и clk_sys на PLL.  
// false — LOCK не получен; clk_sys оставлен на XOSC (безопасно,  
// SCK флэша в норме), s_clk_sys_hz = xosc_hz.  
static bool __not_in_flash_func(rollback_nominal)(  
    const quad_vfo_pll_t *nominal, uint64_t xosc_hz) {  
  
  uint32_t ints = save_and_disable_interrupts();  
  clk_sys_to_xosc(xosc_hz);  
  
  bool ok = pll_setup_bounded(nominal->refdiv, nominal->fbdiv,  
                              nominal->p1, nominal->p2);  
  if (ok) {  
    clk_sys_to_pll(nominal->clk_sys_hz);  
    s_clk_sys_hz = nominal->clk_sys_hz;  
  } else {  
    // Шина на XOSC — система жива, сообщаем реальную частоту.  
    s_clk_sys_hz = (uint32_t)xosc_hz;  
  }  
  restore_interrupts(ints);  
  
  if (!ok)  
    Serial.println("[QUAD] rollback: номинальный PLL без LOCK — "  
                   "шина на XOSC!");  
  return ok;  
}  
  
// ---------------------------------------------------------------------------  
// Перепрограммирование pll_sys по профилю cfg.  
// vreg до перехода при разгоне, после — при понижении. Отход на XOSC,  
// настройка PLL с контролируемым LOCK, SSI BAUDR, переключение clk_sys,  
// верификация frequency_count_khz с 3 ретраями, откат на номинал при  
// провале. false — целевая частота НЕ подтверждена.  
// ---------------------------------------------------------------------------  
bool __not_in_flash_func(quad_vfo_set_clk_sys)(  
    const quad_vfo_pll_t *cfg_in, uint32_t vsel,  
    const quad_vfo_pll_t *nominal, uint64_t xosc_hz) {  
  
  quad_vfo_pll_t cfg = *cfg_in;  
  if (cfg.refdiv == 0) cfg.refdiv = 1;  
  // p1=1 недокументированно делит пополам на VCO < ~2.8 ГГц — отклоняем.  
  if (cfg.p1 < 2) cfg = *nominal;  
  
  uint32_t target_clk_hz =  
      (uint32_t)(((uint64_t)cfg.fbdiv * xosc_hz)  
                 / (uint64_t)(cfg.refdiv * cfg.p1 * cfg.p2));  
  bool is_oc = (target_clk_hz > s_clk_sys_hz);  
  
  if (is_oc) {  
    vreg_set_voltage((vreg_voltage)vsel);  
    busy_wait_us(500);  
  }  
  
  detach_peri();  
  uint32_t ints_status = save_and_disable_interrupts();  
  
  clk_sys_to_xosc(xosc_hz);  
  
  if (!pll_setup_bounded(cfg.refdiv, cfg.fbdiv, cfg.p1, cfg.p2)) {  
    // Целевой PLL мёртв — откат пока прерывания выключены (rollback  
    // сама управляет IRQ). VREG не трогаем — номинал работает на любом.  
    restore_interrupts(ints_status);  
    rollback_nominal(nominal, xosc_hz);  
    if (!is_oc) {  
      busy_wait_us(100);  
      vreg_set_voltage((vreg_voltage)vsel);  
    }  
    return false;  
  }  
  
  // SSI BAUDR: такт флэша <= 48 МГц, чётный делитель 2..34.  
  const uint32_t FLASH_SAFE_MAX_HZ = 48000000u;  
  uint32_t ssi_baud = (uint32_t)((target_clk_hz + FLASH_SAFE_MAX_HZ - 1)  
                               / FLASH_SAFE_MAX_HZ);  
  ssi_baud = (ssi_baud + 1u) & ~1u;  
  if (ssi_baud < 2u)  ssi_baud = 2u;  
  if (ssi_baud > 34u) ssi_baud = 34u;  
  ssi_hw->ssienr = 0;  
  ssi_hw->baudr  = ssi_baud;  
  ssi_hw->ssienr = 1;  
  
  clk_sys_to_pll(target_clk_hz);  
  restore_interrupts(ints_status);  
  
  // Самопроверка: при промахе повторяем весь цикл до 3 раз.  
  bool     locked_ok  = false;  
  uint32_t meas_khz   = 0;  
  uint32_t expect_khz = cfg.clk_sys_hz / 1000u;  
  
  for (int attempt = 0; attempt < 3; attempt++) {  
    busy_wait_us(300);  
    meas_khz = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS);  
    if (meas_khz > expect_khz * 997u / 1000u &&  
        meas_khz < expect_khz * 1003u / 1000u) {  
      locked_ok = true;  
      break;  
    }  
    uint32_t ints2 = save_and_disable_interrupts();  
    clk_sys_to_xosc(xosc_hz);  
    if (!pll_setup_bounded(cfg.refdiv, cfg.fbdiv, cfg.p1, cfg.p2)) {  
      restore_interrupts(ints2);  
      break;                          // PLL мёртв — ретраить бессмысленно  
    }  
    clk_sys_to_pll(target_clk_hz);  
    restore_interrupts(ints2);  
  }  
  
  if (!locked_ok) {  
    // Откат — rollback_nominal сама выключает/включает прерывания.  
    rollback_nominal(nominal, xosc_hz);  
    if (!is_oc) {  
      busy_wait_us(100);  
      vreg_set_voltage((vreg_voltage)vsel);  
    }  
    return false;  
  }  
  
  if (!is_oc) {  
    busy_wait_us(100);  
    vreg_set_voltage((vreg_voltage)vsel);  
  }  
  s_clk_sys_hz = meas_khz * 1000u;    // измеренная реальность  
  return true;  
}