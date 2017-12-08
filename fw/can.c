/*
 * CAN communication handling
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
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/syscfg.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/can.h>

#include "uxb_slave.h"
#include <pb_decode.h>
#include <pb_encode.h>
#include "can.pb.h"

#include "can.h"


struct can_buffer receive_buffer;


void buffer_init(struct can_buffer *buf) {
	memset(buf, 0, sizeof(struct can_buffer));
}


size_t buffer_available(struct can_buffer *buf) {
	if (buf->wp < buf->rp) {
		return buf->rp - buf->wp;
	} else {
		return CAN_BUFFER_SIZE - buf->wp + buf->rp;
	}
}


bool buffer_write(struct can_buffer *buf, const uint8_t *data, uint8_t len, uint32_t id) {
	if (buffer_available(buf) > 0) {
		memcpy(buf->buf[buf->wp].data, data, len);
		buf->buf[buf->wp].id = id;
		buf->buf[buf->wp].len = len;
		buf->wp = (buf->wp + 1) % CAN_BUFFER_SIZE;
		return true;
	} else {
		return false;
	}
}


bool buffer_read(struct can_buffer *buf, uint8_t *data, uint8_t *len, uint32_t *id) {
	if ((CAN_BUFFER_SIZE - buffer_available(buf)) > 0) {
		memcpy(data, buf->buf[buf->rp].data, buf->buf[buf->rp].len);
		*len = buf->buf[buf->rp].len;
		*id = buf->buf[buf->rp].id;
		buf->rp = (buf->rp + 1) % CAN_BUFFER_SIZE;
		return true;
	} else {
		return false;
	}
}


void can_setup(void) {
	rcc_periph_clock_enable(RCC_SYSCFG_COMP);
	/* Remap CANRX/CANTX on PA11/PA12 pins. */
	SYSCFG_CFGR1 |= SYSCFG_CFGR1_PA11_PA12_RMP;

	/* CANRX */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO11);
	/* CANTX */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO12);
	gpio_set_af(GPIOA, GPIO_AF4, GPIO11 | GPIO12);

	rcc_periph_clock_enable(RCC_CAN1);
	can_reset(CAN1);
	can_init(CAN1,
		false,
		true,
		false,
		true,
		false,
		false,
		CAN_BTR_SJW_1TQ,
		CAN_BTR_TS1_9TQ,
		CAN_BTR_TS2_6TQ,
		4,
		false,
		false
	);

	can_filter_id_mask_32bit_init(CAN1,
		0,     /* Filter ID */
		0,     /* CAN ID */
		0,     /* CAN ID mask */
		0,     /* FIFO */
		true
	);

	nvic_enable_irq(NVIC_CEC_CAN_IRQ);
	nvic_set_priority(NVIC_CEC_CAN_IRQ, 2 * 16);

	buffer_init(&receive_buffer);

	can_enable_irq(CAN1, CAN_IER_FMPIE0);
}


void cec_can_isr(void) {
	/** @todo check flags */
	uint32_t id;
	bool ext, rtr;
	uint8_t fmi, length, data[8];

	/* Buffer the received message and do nothing. We are waiting
	 * for the master UXB device to poll the receive buffer. */
	can_receive(CAN1, 0, false, &id, &ext, &rtr, &fmi, &length, data, NULL);
	buffer_write(&receive_buffer, data, length, id);
	can_fifo_release(CAN1, 0);
}


void send_received_messages(void) {
	uint8_t tx[64] = {0};

	CanResponse msg = CanResponse_init_zero;
	msg.which_content = CanResponse_received_messages_tag;

	size_t i = 0;
	uint8_t len;
	uint32_t id;
	while (buffer_read(&receive_buffer, msg.content.received_messages.message[i].data.bytes, &len, &id)) {
		msg.content.received_messages.message[i].id = id;
		msg.content.received_messages.message[i].data.size = len;
		i++;
	}
	msg.content.received_messages.message_count = i;
	msg.content.received_messages.messages_left = CAN_BUFFER_SIZE - buffer_available(&receive_buffer);

	pb_ostream_t stream;
        stream = pb_ostream_from_buffer(tx, sizeof(tx));
        if (pb_encode(&stream, CanResponse_fields, &msg)) {
		libuxb_slot_send_data(&can_slot, tx, stream.bytes_written, true);
	}
}


uxb_master_locm3_ret_t uxb_can_slot_received(void *context, uint8_t *buf, size_t len) {
	(void)context;

	CanRequest msg = CanRequest_init_zero;
	pb_istream_t istream;
        istream = pb_istream_from_buffer(buf, len);

	/* Decode the received message and decide what to do. */
	if (pb_decode(&istream, CanRequest_fields, &msg)) {
		if (msg.which_content == CanRequest_query_received_buffer_tag) {
			send_received_messages();
		}
	}

	return UXB_MASTER_LOCM3_RET_OK;
}
