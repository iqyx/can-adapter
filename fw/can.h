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

#pragma once

#include <stdint.h>
#include <stdbool.h>

#define CAN_BUFFER_SIZE 128

struct can_buffer_item {
	uint8_t data[8];
	uint32_t id;
	uint8_t len;
};

struct can_buffer {
	struct can_buffer_item buf[CAN_BUFFER_SIZE];
	size_t wp;
	size_t rp;
};


void can_setup(void);

void buffer_init(struct can_buffer *buf);
size_t buffer_available(struct can_buffer *buf);
bool buffer_write(struct can_buffer *buf, const uint8_t *data, uint8_t len, uint32_t id);
bool buffer_read(struct can_buffer *buf, uint8_t *data, uint8_t *len, uint32_t *id);
uxb_master_locm3_ret_t uxb_can_slot_received(void *context, uint8_t *buf, size_t len);
void send_received_messages(void);
