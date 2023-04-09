/*
 * wio-e5.h
 *
 *  Created on: Apr 9, 2023
 *      Author: 33768
 */

#ifndef INC_WIO_E5_H_
#define INC_WIO_E5_H_

#include "stm32g4xx_hal.h"

typedef struct{

	//Store the last 200 char receive
	uint8_t last_rx[200];

	uint8_t last_tx[200];


}TELEMETRY_t;



#endif /* INC_WIO_E5_H_ */
