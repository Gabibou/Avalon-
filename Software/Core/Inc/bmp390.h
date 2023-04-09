/*
 * bmp390.h
 *
 *  Created on: Apr 9, 2023
 *      Author: 33768
 */

#ifndef INC_BMP390_H_
#define INC_BMP390_H_

#include "stm32g4xx_hal.h"

typedef struct{
	//pressure measured by the sensor
	uint32_t pressure_hpa;
	//reference altitude (where we calibrated the sensor)
	//Should often be done at ground level
	float altitude_ref;

	float ground_altitude;


}ALTIMETER_t;



#endif /* INC_BMP390_H_ */
