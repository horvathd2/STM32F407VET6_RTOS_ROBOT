/*
 * usb_packet.h
 *
 *  Created on: Dec 2, 2025
 *      Author: H.Dani
 */

#ifndef INC_USB_PACKET_H_
#define INC_USB_PACKET_H_

#include <stdint.h>

#define PACKET_SIZE 11

typedef struct __attribute__((packed)) {
	uint8_t str;
	uint8_t cmd;
	int16_t sp1;
	int16_t sp2;
	int16_t sp3;
	int16_t sp4;
	uint8_t chksum;
} SetpointPacket;

typedef enum {
	CMD_NOP     = 0x00,   // do nothing
	CMD_CONN	= 0x01,   // start motion loop
	CMD_DISC    = 0x02,   // stop motion loop
	CMD_START   = 0x03,   // start motion loop
	CMD_STOP    = 0x04,   // stop motion loop
	CMD_HOME    = 0x05,   // begin homing routine
	CMD_ZERO    = 0x06,   // zero encoder
	CMD_SETSP   = 0x07    // update setpoints only
} CommandType;

extern volatile SetpointPacket active_packet;

#endif /* INC_USB_PACKET_H_ */
