/*
 * l80-m39.h
 *
 *  Created on: Apr 9, 2023
 *      Author: 33768
 */

#ifndef SRC_L80_M39_H_
#define SRC_L80_M39_H_

#include "stm32g4xx_hal.h"

typedef struct{

	float latitude_deg_s;

	float longitude_deg_s;

	float altitude_deg_s;

	//If North then 0 / south 1
	float north_south;
	//If east then 0 / west 1
	float east_west;

	float speed;

	float utc_time;

	uint8_t satelite_number;


}GPS_t;

#endif /* SRC_L80_M39_H_ */
