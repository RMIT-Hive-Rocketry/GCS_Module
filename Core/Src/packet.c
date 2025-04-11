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

		msg.id = id;
		msg.data[idx] = state; //state is data[0]
		memcpy(&msg.data[idx++], ~state, sizeof(state)); //store the inverse of state data[1]

		return msg;
	}


	SX1272_Packet GCS_TO_AV_CMD(
			uint8_t id,
			uint8_t state,
			uint8_t broadcast_cmd)
	{
		SX1272_Packet msg;
		int idx = 0;

		msg.id = id;
		msg.data[idx] = state; //state is data[0]
		memcpy(&msg.data[idx++], ~state, sizeof(state));
		memcpy(&msg.data[idx++], broadcast_cmd, sizeof(broadcast_cmd));

		return msg;
	}




