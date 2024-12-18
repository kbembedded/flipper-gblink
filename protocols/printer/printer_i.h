// SPDX-License-Identifier: BSD-2-Clause
// Copyright (c) 2024 KBEmbedded

#ifndef PRINTER_I_H
#define PRINTER_I_H

#include <furi.h>
#include <protocols/printer/include/printer_proto.h>

#define START_L_BYTE		0x88
#define START_H_BYTE		0x33
#define ALIVE_BYTE		0x81

#define CMD_INIT		0x01
#define CMD_PRINT		0x02
#define CMD_TRANSFER		0x10 // Custom command from Photo! ROM
#define CMD_DATA		0x04
#define CMD_BREAK		0x08 // Interrupt printing in progress
#define CMD_STATUS		0x0F // Also referred to as a NUL packet

/* These constants are private to the printer protocol */
#define LINE_BUF_SZ		640 // (TILE_SIZE * WIDTH * 2)
#define TRANSFER_SZ		3584 // (16*16*14) // Image minus frame

/* Hooray! Real numbers and not using guesses made by other GB printer projects!
 *
 * From https://ia903208.us.archive.org/9/items/GameBoyProgManVer1.1/GameBoyProgManVer1.1.pdf
 *
 * There are a couple of different timeouts used for the printer.
 * Inter-byte timeouts:
 * 	An interval of 270 us to 5 ms must be interposed between each byte.
 * 	XXX: I am unsure what happens at this time when these are exceeded
 *
 * Inter-packet timeouts:
 * 	An interval of 270 us to 117 ms must be allowed between the transfer
 * 	of each packet.
 * 	XXX: From observations, need to confirm, when exceeded this will
 * 	reset the printer state. Need to confirm in manual
 *
 * Syncronization/pings:
 * 	Once connection is confirmed (XXX: What does that mean?), the printer
 * 	must be sent STATUS (aka NUL) packet every 100 ms. If this is exceeded
 * 	(as noted above) the printer will return to an initialized state (XXX: Which state is that)
 * 	XXX: In practice, this seems to only matter when communicating with the
 * 	printer, i.e. waiting for a print to complete can simply stop, but,
 * 	that is probably not the best choice.
 *
 *
 * Errors:
 * Status byte is as follows:
 * 	bit 7: 1 == Low battery
 * 	bit 6: 1 == Other Error
 * 	bit 6: 1 == Paper jam (abnormal motor operation)
 * 	bit 4: 1 == Packet error
 * 	bit 3: 1 == Unprocessed data in printer RAM
 * 	bit 2: 1 == Image Data Full
 * 	bit 1: 1 == Printer Busy
 * 	bit 0: 1 == Checksum error 
 * The GB programming manual establishes some standard error codes:
 *	Low battery == Error No. 1
 *	Status bytes 0xff 0xff == Error No. 2 (Essentially, no printer connected)
 *	Paper Jam/abnormal motor operation == Error No. 3
 *	Other error == Error No. 4
 * A value other than ALIVE_BYTE means a device that is not a printer is connected,
 * this should be conveyed in an error message.
 * Error recovery must sever communication with printer, and inform the user.
 *
 * The ALIVE_BTYE MSB is technically the "device number." The MSB is always set
 * and the lower bits indicate device number, the printer is device number 1.
 * I would guess that this was intended for other future devices to be able
 * to work with this protocol, but nothing was ever created.
 *
 * Packet format
 * 	Preamble	Header									Data		Checksum			Footer
 * From GB:
 * 	0x88,	0x31,	Type,	Compress,	LSB data byte count,	MSB data byte count,	Data byte(s),	LSB CKSUM,	MSB CKSUM,	Dummy,	Dummy
 * From Printer:
 * 	NULL,	NULL,	NULL,	NULL,		NULL,			NULL,			NULL,		NULL		NULL,		0x81,	STATUS
 *
 * Command/packet details:
 * CMD_INIT:
 * 	Used to init printer and check connection. Must be sent first. Resets
 * 	transferred data held in RAM.
 *
 * 	Has no data bytes
 *
 * CMD_PRINT:
 * 	Used to initiate a print from the current buffer.
 *
 * 	Data bytes:
 * 	Byte 1: Number of sheets to print, 0-255, 0 == line feed only
 * 		- Link partner need to be able to implement CMD_BREAK to be able
 * 		to halt a print job in progress!
 * 		- If CMD_PRINT is sent within 100 ms of the motor being stopped
 * 		(CMD_BREAK), resume may be incorrect. Wait at least 100 ms after
 * 		the motor has been stopped!
 * 	Byte 2: Number of line feeds, upper 4 bits feeds before printing,
 * 		lower 4 bits feeds after printing; both values 0-15.
 * 		- Each feed is 2.64 mm in length
 * 		- Always set number of line feeds before printing to 1 or
 * 		greater and after printing to 3 or greater except when doing
 * 		a continuous, multi-image print with both values as 0 for in
 * 		between feeds. Other values for parameters may result in double
 * 		printing o the same line or failure of the last line to reach
 * 		the paper cutter.
 * 	Byte 3: Palette values, basically useless for the printer, but, could
 *		be used when printing to something else. XXX: Consider implementing
 *		this in some way?
 *	Byte 4: Print _density_ XXX: Is that a typo, or is every other RE doc wrong
 *		in calling it intensity?
 *		0-127, -25%  0%    +25%
 *		       0x00  0x40  0x7F
 * CMD_DATA:
 * 	Compression seems to (sensibly) only be applicable on data packets.
 *
 * 	
 *
 */

/* NOTE:
 * These numbers are empirically gathered from a few different games thus far.
 * There are many notes floating around the internet of the GB Printer having
 * a 100 ms limit between packets where it will reset. However, I've seen
 * Pokemon Pinball wait 99.5 ms between packets after a print command which is
 * a bit too close for comfort. As this code tracks timestamps _after_ each byte,
 * that ends up just over 110 ms which trips the hard timeout and resets state.
 * This often cofuses the hell out of games.
 *
 * Additionally, on the other end of the spectrum, Pokemon Gold absolutely uses
 * the hard timeout to reset the printer between packets. It waits ~278 ms, when
 * it could just send an init command.
 *
 * Even more silly, Pokemon Pinball has a fun quirk where if the print completes
 * immediately (usually the Flipper will mark a print complete with a single
 * packet turnaround), it asks for status a couple of times, then starts (presumably)
 * another status packet, but the second byte in the transfer is stopped mid-byte.
 * Between that point and the start of the next, real packet, is 30 ms, 32.6 ms
 * if you go from end of last byte received to start of next, real packet.
 *
 * This means there is some "soft" timeout that the printer uses to reset a packet
 * transfer in progress, but don't reset the whole printer state.
 *
 * There are wisps of some "unknown" bit timeout of 1.49 ms. But I've not yet
 * seen that in action.
 *
 * As far as I know, no one has dumped and reverse engineered the ROM of the
 * Game Boy Printer directly. I think all of the existing documentation was from
 * reverse engineering the communication channel. Maybe someday I'll dump the
 * GB Printer ROM and try to better understand all of it.
 *
 * Additionally, the gameboy camera seems to hint at a 2 second busy timeout, that
 * is that the printer needs to return a printing status within 2 seconds of starting
 * the print. There is also a hint of a 20 second print timeout, that is that the
 * printer needs to return print complete within 20 seconds of starting the print.
 * These two values are not really used in this library yet.
 */
#define HARD_TIMEOUT_US 125000
#define SOFT_TIMEOUT_US 20000

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
	COMPLETE,
};

enum printer_state {
	INIT,
	INIT_STATUS,
	FILL,
	FILL_BLANK,
	PRINT,
	PRINT_STATUS,
	PRINT_COMPLETE,
	NUM_STATES,
};

struct packet {
	uint8_t cmd;
	bool compress;
	uint16_t len; // This is stored in the flipper endianness, arrives LSB first from GB, unmodified in code
	uint8_t line_buf[LINE_BUF_SZ]; // 640 bytes, enough for two lines of tiles
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
	enum printer_state state;
	FuriSemaphore *sem;

	struct gb_image *image; // Details of the current image being sent/received
	size_t image_data_pos;

	FuriThread *thread;
};

#endif // PRINTER_I_H
