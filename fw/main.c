#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/iwdg.h>
#include <libopencm3/stm32/syscfg.h>
#include <libopencm3/stm32/can.h>

#include "uxb_slave.h"
#include "can.h"

#define LED_PORT GPIOA
#define LED_PIN GPIO3

#define PWR_PORT GPIOB
#define PWR_PIN GPIO7


static void clock_setup(void) {
	rcc_osc_on(RCC_HSE);
	rcc_wait_for_osc_ready(RCC_HSE);
	rcc_set_sysclk_source(RCC_HSE);
	rcc_set_hpre(RCC_CFGR_HPRE_NODIV);

	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
}


static void gpio_setup(void) {
	/* Initialize the STAT LED. */
	gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN);

	/* Power pin for the isolated DC-DC converter. */
	gpio_mode_setup(PWR_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PWR_PIN);
}


static void delay(uint32_t l) {
	for (uint32_t i = 0; i < l; i++) {
		__asm__("nop");
	}
}


int main(void) {
	clock_setup();
	gpio_setup();
	uxb_slave_init();
	can_setup();

	gpio_clear(LED_PORT, LED_PIN);

	/* Enable the DC-DC converter. */
	gpio_set(PWR_PORT, PWR_PIN);

	while (1) {
		/* Do nothing. */
		delay(100000);
	}

	return 0;
}


