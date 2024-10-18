// SPDX-License-Identifier: BSD-2-Clause
// Copyright (c) 2024 KBEmbedded

#ifndef PRINTER_I_H
#define PRINTER_I_H

#include <protocols/printer/include/printer_proto.h>

#define START_L_BYTE		0x88
#define START_H_BYTE		0x33
#define ALIVE_BYTE		0x81

#define CMD_INIT		0x01
#define CMD_PRINT		0x02
#define CMD_TRANSFER		0x10
#define CMD_DATA		0x04
#define CMD_STATUS		0x0f

/* These constants are private to the printer protocol */
#define LINE_BUF_SZ		640 // (TILE_SIZE * WIDTH * 2)
#define TRANSFER_SZ		3584 // (16*16*14) // Image minus frame

//GB seems to use 2 second busy timeout? I think that is a go to busy/printing within 2 seconds?
//20 second print timeout

enum packet_state {
	START_L,
	START_H,
	COMMAND,
	COMPRESS,
	LEN_L,
	LEN_H,
	DATA,
	CKSUM_L,
	CKSUM_H,
	ALIVE,
	STATUS,
};

struct packet {
	uint8_t cmd;
	bool compress;
	uint16_t len; // This is stored in the flipper endianness, arrives LSB first from GB, unmodified in code
	uint8_t line_buf[LINE_BUF_SZ]; // 640 bytes, enough for two lines of tiles
	uint16_t cksum; // This is stored in the flipper endianness, arrives LSB first from GB
	uint8_t status;
	
	/* These are not part of the packet, but used by us */
	uint16_t cksum_calc;
	size_t line_buf_sz;
	bool print_complete;
	uint8_t zero_counter;
	enum packet_state state;
	uint32_t time;
};

#define THREAD_FLAGS_EXIT	(1 << 0)
#define THREAD_FLAGS_DATA	(1 << 1)
#define THREAD_FLAGS_PRINT	(1 << 2)
#define THREAD_FLAGS_COMPLETE	(1 << 3)
#define THREAD_FLAGS_ALL	(THREAD_FLAGS_EXIT | THREAD_FLAGS_DATA | THREAD_FLAGS_PRINT | THREAD_FLAGS_COMPLETE)

struct printer_proto {
	void *gblink_handle;

	void (*callback)(void *cb_context, struct gb_image *image, enum cb_reason reason);
	void *cb_context;

	struct packet *packet; //packet data used by send()/receive() for tracking

	struct gb_image *image; // Details of the current image being sent/received

	FuriThread *thread;
};

#endif // PRINTER_I_H
