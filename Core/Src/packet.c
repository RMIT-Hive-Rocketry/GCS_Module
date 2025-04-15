/*
 * packet.c
 *
 *  Created on: Apr 8, 2025
 *      Author: lucas
 */


#include "packet.h"

	SX1272_Packet GCS_TO_GSE_CMD(
		uint8_t id,
		uint8_t state)
	{
		SX1272_Packet msg;

		int idx = 0;

		msg.transmit_buffer[idx] = id;
		msg.transmit_buffer[idx++] = state;
		msg.transmit_buffer[idx++] = ~state;

		return msg;
	}


	SX1272_Packet GCS_TO_AV_CMD(
			uint8_t id,
			uint8_t state,
			uint8_t broadcast_cmd)
	{
		SX1272_Packet msg;
		int idx = 0;

		msg.transmit_buffer[idx] = id;
		msg.transmit_buffer[idx++] = state;
		msg.transmit_buffer[idx++] = ~state;
		msg.transmit_buffer[idx++] = broadcast_cmd;

		return msg;
	}




