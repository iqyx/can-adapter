/*
 * UXB Bus slave communication handling
 *
 * Copyright (C) 2017, Marek Koza, qyx@krtko.org
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdbool.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/exti.h>

#include "libuxb.h"
#include "uxb_slave.h"

#include "can.h"
#include <pb_decode.h>
#include <pb_encode.h>
#include "can.pb.h"


const struct uxb_master_locm3_config uxb_slave_config = {
	.spi_port = SPI1,
	.spi_af = GPIO_AF0,
	.sck_port = GPIOA, .sck_pin = GPIO5,
	.miso_port = GPIOA, .miso_pin = GPIO6,
	.mosi_port = GPIOA, .mosi_pin = GPIO7,
	.frame_port = GPIOA, .frame_pin = GPIO4,
	.id_port = GPIOA, .id_pin = GPIO2,
	.delay_timer = TIM14,
	.delay_timer_freq_mhz = 1,
	.control_prescaler = SPI_CR1_BR_FPCLK_DIV_8,
	.data_prescaler = SPI_CR1_BR_FPCLK_DIV_8,
};


LibUxbBus uxb_bus;
LibUxbDevice uxb_device;

/* Minimum buffer size for a descriptor line is 64 bytes (including the trailing \0). */
LibUxbSlot descriptor_slot;
uint8_t descriptor_slot_buffer[64];

/* CAN interface slot. */
LibUxbSlot can_slot;
uint8_t can_slot_buffer[128];

const uint8_t master_address[] = {0};

/* Default interface address. It will be copied from the MCU unique identity
 * during initialization. */
uint8_t device_address[] = {0, 0, 0, 0, 0, 0, 0, 13};

#define UXB_DESCRIPTOR_SIZE 6
const char *uxb_descriptor[] = {
	"device=can-adapter",
	"hw-version=1.0.0+20171011",
	"fw-version=0.1.0",
	"cspeed=1",
	"dspeed=1",
	"slot=1,can.1.0.0",
};


static uxb_master_locm3_ret_t uxb_read_descriptor(void *context, uint8_t *buf, size_t len) {
	(void)context;

	uint8_t zero = 0;

	if (len != 1) {
		return UXB_MASTER_LOCM3_RET_FAILED;
	}
	if (buf[0] == 0) {
		/* Send the 0 back. */
		libuxb_slot_send_data(&descriptor_slot, &zero, 1, true);
	} else {
		uint8_t descriptor_index = buf[0] - 1;
		if (descriptor_index >= UXB_DESCRIPTOR_SIZE) {
			libuxb_slot_send_data(&descriptor_slot, &zero, 1, true);
		} else {
			libuxb_slot_send_data(&descriptor_slot, (uint8_t *)uxb_descriptor[descriptor_index], strlen(uxb_descriptor[descriptor_index]) + 1, true);
		}
	}

	return UXB_MASTER_LOCM3_RET_OK;
}


uxb_slave_ret_t uxb_slave_init(void) {
	/* Initialize the UXB bus. */
	rcc_periph_clock_enable(RCC_SPI1);
	rcc_periph_clock_enable(RCC_TIM14);

	/* Setup a timer for precise UXB protocol delays. */
	timer_reset(TIM14);
	timer_set_mode(TIM14, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_continuous_mode(TIM14);
	timer_direction_up(TIM14);
	timer_disable_preload(TIM14);
	timer_enable_update_event(TIM14);
	timer_set_prescaler(TIM14, (rcc_ahb_frequency / 1000000) - 1);
	timer_set_period(TIM14, 65535);
	timer_enable_counter(TIM14);

	libuxb_bus_init(&uxb_bus, &uxb_slave_config);

	libuxb_device_init(&uxb_device);
	libuxb_device_set_address(&uxb_device, device_address, master_address);
	libuxb_bus_add_device(&uxb_bus, &uxb_device);

	libuxb_slot_init(&can_slot);
	libuxb_slot_set_slot_number(&can_slot, 1);
	libuxb_slot_set_slot_buffer(&can_slot, can_slot_buffer, sizeof(can_slot_buffer));
	libuxb_slot_set_data_received(&can_slot, uxb_can_slot_received, NULL);
	libuxb_device_add_slot(&uxb_device, &can_slot);

	libuxb_slot_init(&descriptor_slot);
	libuxb_slot_set_slot_number(&descriptor_slot, 0);
	libuxb_slot_set_slot_buffer(&descriptor_slot, descriptor_slot_buffer, sizeof(descriptor_slot_buffer));
	libuxb_slot_set_data_received(&descriptor_slot, uxb_read_descriptor, NULL);
	libuxb_device_add_slot(&uxb_device, &descriptor_slot);

	/* Setup uxb exti interrupts. */
	nvic_enable_irq(NVIC_EXTI4_15_IRQ);
	/* The priority must be set fairly high in order not to interrupt the UXB communication. */
	nvic_set_priority(NVIC_EXTI4_15_IRQ, 1 * 16);

	rcc_periph_clock_enable(RCC_SYSCFG_COMP);
	exti_select_source(EXTI4, GPIOA);
	exti_set_trigger(EXTI4, EXTI_TRIGGER_FALLING);
	exti_enable_request(EXTI4);

	return UXB_SLAVE_OK;
}


void exti4_15_isr(void) {
	// rcc_set_hpre(RCC_CFGR_HPRE_NODIV);
	exti_reset_request(EXTI4);
	exti_disable_request(EXTI4);
	libuxb_bus_frame_irq(&uxb_bus);
	exti_enable_request(EXTI4);
	// rcc_set_hpre(RCC_CFGR_HPRE_DIV4);
}
