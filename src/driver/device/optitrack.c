#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "stm32f4xx_conf.h"
#include "uart.h"
#include "led.h"
#include "optitrack.h"

#define OPTITRACK_SERIAL_MSG_SIZE 32

volatile int optitrack_buf_pos = 0;
uint8_t optitrack_buf[OPTITRACK_SERIAL_MSG_SIZE] = {0};

optitrack_t optitrack;

void optitrack_buf_push(uint8_t c)
{
	if(optitrack_buf_pos >= OPTITRACK_SERIAL_MSG_SIZE) {
		/* drop the oldest data and shift the rest to left */
		int i;
		for(i = 1; i < OPTITRACK_SERIAL_MSG_SIZE; i++) {
			optitrack_buf[i - 1] = optitrack_buf[i];
		}

		/* save new byte to the last array element */
		optitrack_buf[OPTITRACK_SERIAL_MSG_SIZE - 1] = c;
		optitrack_buf_pos = OPTITRACK_SERIAL_MSG_SIZE;
	} else {
		/* append new byte if the array boundary is not yet reached */
		optitrack_buf[optitrack_buf_pos] = c;
		optitrack_buf_pos++;
	}
}

void optitrack_handler(uint8_t c)
{
	optitrack_buf_push(c);
	if(c == '+' && optitrack_buf[0] == '@') {
		/* decode optitrack message */
		led_on(LED_G);
#if 1
		if(optitrack_serial_decoder(optitrack_buf) == 0) {
			optitrack_buf_pos = 0; //reset position pointer
		}
#endif
	}
}

#define OPTITRACK_CHECKSUM_INIT_VAL 19
static uint8_t generate_optitrack_checksum_byte(uint8_t *payload, int payload_count)
{
	uint8_t result = OPTITRACK_CHECKSUM_INIT_VAL;

	int i;
	for(i = 0; i < payload_count; i++)
		result ^= payload[i];

	return result;
}

int optitrack_serial_decoder(uint8_t *buf)
{
	uint8_t recv_checksum = buf[1];
	uint8_t checksum = generate_optitrack_checksum_byte(&buf[3], OPTITRACK_SERIAL_MSG_SIZE - 4);
	if(checksum != recv_checksum) {
		return 1; //error detected
	}

	optitrack.mav_id = buf[2];
	memcpy(&optitrack.pos_x, &buf[3], sizeof(float));
	memcpy(&optitrack.pos_y, &buf[7], sizeof(float));
	memcpy(&optitrack.pos_z, &buf[11], sizeof(float));
	memcpy(&optitrack.quat_x, &buf[15], sizeof(float));
	memcpy(&optitrack.quat_z, &buf[19], sizeof(float));
	memcpy(&optitrack.quat_y, &buf[23], sizeof(float));
	memcpy(&optitrack.quat_w, &buf[27], sizeof(float));

	return 0;
}
