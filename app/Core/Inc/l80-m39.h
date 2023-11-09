/*
 * l80-m39.h
 *
 *  Created on: Apr 9, 2023
 *      Author: 33768
 */

#ifndef SRC_L80_M39_H_
#define SRC_L80_M39_H_

#include "stm32g4xx_hal.h"

#define BUFFER_SIZE_NMEA 1000

typedef struct{

	uint8_t hour;
	uint8_t	minute;
	uint8_t second;

}TIME_t;

typedef enum{
	north,south,east,west
}COMPAS_t;

typedef enum{
	unvalid,fix_gps,fix_dgps
}FIX_t;


typedef struct{

	float latitude_deg_s;

	float longitude_deg_s;

	float altitude_deg_s;

	float altitude_correction;

	COMPAS_t north_south;

	COMPAS_t east_west;

	float speed;

	TIME_t utc_time;

	uint8_t satelite_number;

	FIX_t qualification;
}GPS_t;


void gps_ProcessFix(uint8_t utc_incomming[],GPS_t *gps_struct);
void gps_ReadNMEA(uint8_t nmea_data[],GPS_t *gps_struct);
void gps_ProcessUTC(uint8_t utc_incomming[],GPS_t *gps_struct);
void gps_ProcessLatitude(uint8_t utc_incomming[],GPS_t *gps_struct);
void gps_ProcessLongitude(uint8_t utc_incomming[],GPS_t *gps_struct);
void gps_ProcessNorthSouth(uint8_t utc_incomming[],GPS_t *gps_struct);
void gps_ProcessEastWest(uint8_t utc_incomming[],GPS_t *gps_struct);
void gps_ProcessEastWest(uint8_t utc_incomming[],GPS_t *gps_struct);
void gps_ProcessSatelliteCount(uint8_t utc_incomming[],GPS_t *gps_struct);
void gps_ProcessAltitude(uint8_t utc_incomming[],GPS_t *gps_struct);
void gps_ProcessAltitudeCorre(uint8_t utc_incomming[],GPS_t *gps_struct);
#endif /* SRC_L80_M39_H_ */
