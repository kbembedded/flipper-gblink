// SPDX-License-Identifier: BSD-2-Clause
// Copyright (c) 2024 KBEmbedded

#include <stdint.h>

#include <furi.h>

#include <gblink/include/gblink.h>
#include <protocols/printer/include/printer_proto.h>
#include "printer_i.h"

#define TAG "printer_send"

/* The reset doesn't need to copy any data since that will be handled in
 * the main callback
 *
 * XXX: I think this should be packet reset?
 */
static void printer_reset(struct printer_proto *printer)
{
	/* Clear out the current packet data */
	memset(printer->packet, '\0', sizeof(struct packet));

	/* This is technically redundant, done for completeness */
	printer->packet->state = START_L;

	/* Packet timeout start */
	printer->packet->time = DWT->CYCCNT;
}

static void byte_callback(void *context, uint8_t val)
{
	UNUSED(val);
	struct printer_proto *printer = context;
	struct packet *packet = printer->packet;
	const uint32_t time_ticks = furi_hal_cortex_instructions_per_microsecond() * HARD_TIMEOUT_US;
	uint8_t data_out = 0x00;

	if ((DWT->CYCCNT - packet->time) > time_ticks) {
		printer_reset(printer);
		/* XXX: HACK: on a hard timeout, force restart the whole chain
		 * of commands. Not sure if this will work the way I expect it
		 * to or if there will be other issues as well. This may
		 * actually never even get tripped.
		 */
		printer->state = INIT;
	}

	if ((DWT->CYCCNT - packet->time) > furi_hal_cortex_instructions_per_microsecond() * SOFT_TIMEOUT_US)
		packet->state = START_L;

	/* Packet timeout restart */
	packet->time = DWT->CYCCNT;

	/* TODO: flash led? */

	switch (packet->state) {
	case START_L:
		data_out = START_L_BYTE;
		packet->state = START_H;
		break;
	case START_H:
		data_out = START_H_BYTE;
		packet->state = COMMAND;
		break;
	case COMMAND:
		switch (printer->state) {
		default:
		case INIT:
			packet->cmd = CMD_INIT;
			data_out = CMD_INIT;
			packet->len = 0;
			break;
		case FILL:
			packet->cmd = CMD_DATA;
			data_out = CMD_DATA;
			if ((printer->image->data_sz - printer->image_data_pos) >= LINE_BUF_SZ)
				packet->len = LINE_BUF_SZ;
			else
				packet->len = (printer->image->data_sz - printer->image_data_pos);
			/* TODO: XXX: I think this can go away and only reset at end of packet */
			packet->line_buf_sz = 0;
			memcpy(packet->line_buf, &printer->image->data[printer->image_data_pos], packet->len);
			printer->image_data_pos += packet->len;
			break;
		case FILL_BLANK:
			packet->cmd = CMD_DATA;
			data_out = CMD_DATA;
			packet->len = 0;
			break;
		case PRINT:
			packet->cmd = CMD_PRINT;
			data_out = CMD_PRINT;
			packet->len = 4;
			packet->line_buf[0] = printer->image->num_sheets;
			packet->line_buf[1] = printer->image->margins;
			packet->line_buf[2] = printer->image->palette;
			packet->line_buf[3] = printer->image->exposure;
			break;
		case PRINT_STATUS:
		case INIT_STATUS:
			packet->cmd = CMD_STATUS;
			data_out = CMD_STATUS;
			packet->len = 0;
			break;
		case PRINT_COMPLETE:
			break;
		}
		packet->state = COMPRESS;
		/* The command byte is the first one added to the checksum */
		packet->cksum_calc = data_out;
		break;
	case COMPRESS:
		data_out = 0; // No compression
		packet->cksum_calc += data_out;
		packet->state = LEN_L;
		break;
	case LEN_L:
		data_out = (packet->len & 0xff);
		packet->cksum_calc += data_out;
		packet->state = LEN_H;
		break;
	case LEN_H:
		data_out = ((packet->len >> 8) & 0xff);
		packet->cksum_calc += data_out;

		if (packet->len) {
			packet->state = DATA;
		} else {
			packet->state = CKSUM_L;
		}
		break;
	case DATA:
		data_out = packet->line_buf[packet->line_buf_sz];
		packet->cksum_calc += data_out;
		packet->line_buf_sz++;
		if (packet->line_buf_sz == packet->len)
			packet->state = CKSUM_L;
		break;
	case CKSUM_L:
		data_out = (packet->cksum_calc & 0xff);
		packet->state = CKSUM_H;
		break;
	case CKSUM_H:
		data_out = ((packet->cksum_calc >> 8) & 0xff);
		packet->state = ALIVE;
		break;
	case ALIVE:
		data_out = 0;
		packet->state = STATUS;
		break;
	case STATUS:
		data_out = 0;
		packet->state = COMPLETE;
		break;
	/* Most of the time this will be the last state and we send out no more
	 * data and simply release the semaphore. The only time we would set up
	 * the next byte is if there is still data to send, e.g. in the case
	 * of multiple FILL packets needing to be sent to transfer image data.
	 */
	case COMPLETE:
		packet->status = val;

		if ((packet->status & STATUS_ERR_MASK) == 0) {
			switch (packet->cmd) {
			case CMD_INIT:
				break;
			case CMD_DATA:
				/* This will be true for both image data as well as the
				 * blank fill command that happens.
				 */
				/* TODO: XXX: Test if, rather than this convoluted
				 * "send next start byte and end up in a weird place
				 * in the state machine," if its possible to instead just
				 * send a 0x00 byte via transfer and then let the whole
				 * state machine loose again from the start.
				 */
				furi_thread_flags_set(printer->thread, THREAD_FLAGS_DATA);
				if (printer->image_data_pos < printer->image->data_sz) {
					data_out = START_L_BYTE;
					packet->state = START_H;
					goto more_data;
				}
				break;
			case CMD_PRINT:
				furi_thread_flags_set(printer->thread, THREAD_FLAGS_PRINT);
				break;
			case CMD_STATUS:
				furi_delay_ms(10);
				/* TODO: There is a 10 ms timeout after PRINT_STATUS and 35 ms after INIT_STATUS */
				/* TODO: check status and wait to go from printing to not printing */
			}
		}


		/* TODO: NOTE: XXX:
		 * Need to give the semaphore at the end of a packet. Be it a
		 * completion or an error.
		 */
		/* We're done, don't transfer the next byte */
		furi_semaphore_release(printer->sem);
		return;
	default:
		FURI_LOG_E(TAG, "unknown status!");
		break;
	}

more_data:
	/* transfer next byte */
	/* Wait a bit between bytes */
	furi_delay_us(300);
	gblink_transfer(printer->gblink_handle, data_out);
	return;
}

void printer_send_start(void *printer_handle, struct gb_image *image)
{
	struct printer_proto *printer = printer_handle;

	printer->sem = furi_semaphore_alloc(1, 0);

	/* Set up defaults the send path needs */
	gblink_callback_set(printer->gblink_handle, byte_callback, printer);
	gblink_clk_source_set(printer->gblink_handle, GBLINK_CLK_INT);

	printer_reset(printer);

	memcpy(printer->image, image, sizeof(*image));
	printer->image_data_pos = 0;

	gblink_start(printer->gblink_handle);

	/* Start the transmission loop
	 * This blocks until complete or error
	 */
	for (printer->state = INIT; printer->state < NUM_STATES; printer->state++) {
		printer_reset(printer);
		byte_callback(printer, 0);
		furi_semaphore_acquire(printer->sem, FuriWaitForever);
		FURI_LOG_D("send", "pos %d, sz %d", printer->image_data_pos, printer->image->data_sz);
		furi_delay_us(560);
		if (printer->state == PRINT_STATUS) {
			if (printer->packet->status & STATUS_PRINTING)
				printer->state--; // Rerun the PRINT_STATUS state until we're dong printing
		}
		/* TODO: Check printer->packet->status? */
	}

	/* TODO: Return a value */
}
