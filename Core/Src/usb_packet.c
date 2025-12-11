/*
 * usb_packet.c
 *
 *  Created on: Dec 2, 2025
 *      Author: H.Dani
 */

#include "usb_packet.h"

volatile SetpointPacket active_packet = {0};
uint8_t rxBuffer[64];
uint8_t rxIndex = 0;
