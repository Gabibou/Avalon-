/*
 * l80-m39.c
 *
 *  Created on: Apr 9, 2023
 *      Author: 33768
 */

#include "l80-m39.h"


void gps_ReadNMEA(uint8_t nmea_data[],GPS_t *gps_struct){

	uint8_t string_nmea[6];
	uint8_t gpgga_nmea[6] = "GPGGA\0";
	uint8_t readed_data[12];
	uint8_t delta;
	uint8_t begin_offset;
	uint8_t processing_offset = 0;

	//Loop for all the nmea data
	for(int i=0;i<BUFFER_SIZE_NMEA;i++){
		/*only look for a data beginin symbol = $ */
		if(nmea_data[i] == '$'){
			/*Check if we have $GPGGA*/
			for(int j=0;j<5;j++){
				string_nmea[j] = nmea_data[(i+j+1)];
			}
			string_nmea[5] = 0;
			/*Be aware that input string of strcmp need to end on \0 If not the result will probably be random value*/
			if(strcmp(string_nmea,gpgga_nmea) == 0){
				i+=6;	/*Skip GPGGA,*/
				for(int k=0;k<12;k++){

					/*Read a part of the incomming data*/
					i++;	/*skip ','*/
					begin_offset = i;
					delta = 0;

					while((nmea_data[i] !='\0' )&&( nmea_data[i] != ',')){
						readed_data[delta] = nmea_data[i];
						i++;
						delta = i - begin_offset;
					}
					processing_offset++;
					/*Process incomming data*/
					switch (processing_offset) {
						/*UTC*/
						case 1:
							gps_ProcessUTC(readed_data, gps_struct);
							break;

						/*LATITUDE*/
						case 2:
							gps_ProcessLatitude(readed_data, gps_struct);
							break;

						/*NORTH - SOUTH*/
						case 3:
							gps_ProcessNorthSouth(readed_data, gps_struct);
							break;

						/*LONGITUDE*/
						case 4:
							gps_ProcessLongitude(readed_data, gps_struct);
							break;

						/*EAST - WEST*/
						case 5:
							gps_ProcessEastWest(readed_data,gps_struct);
							break;

						/*FIX QUALIFICATION*/
						case 6:
							gps_ProcessFix(readed_data, gps_struct);
							break;

						/*SATELLITES COUNT */
						case 7:
							gps_ProcessSatelliteCount(readed_data, gps_struct);
							break;

						/*MSL ALTITUDE*/
						case 9:
							gps_ProcessAltitude(readed_data, gps_struct);
							break;

						/*ELLIPTICAL CORRECTION */
						case 11:
							gps_ProcessAltitudeCorre(readed_data, gps_struct);	//Can be use to process altitude correction as well
							break;
					}
				}
				/*After the first $GPGGA read we need to quit the current reading process*/
				break;

			}
		}
	}
}

void gps_ProcessUTC(uint8_t utc_incomming[],GPS_t *gps_struct){
	gps_struct->utc_time.hour = ((utc_incomming[0] - '0')*10) + (utc_incomming[1] - '0');
	gps_struct->utc_time.minute = ((utc_incomming[2] - '0')*10) + (utc_incomming[3] - '0');
	gps_struct->utc_time.second = ((utc_incomming[4] - '0')*10) + (utc_incomming[5] - '0');
}

void gps_ProcessLatitude(uint8_t utc_incomming[],GPS_t *gps_struct){

	int8_t degree_DMm = 0;
	float minute_DMm = 0;

	/*Add reader degree*/
	degree_DMm = ((utc_incomming[0] - '0')*10) + (utc_incomming[1] - '0');
	/*Add first part of minutes*/
	minute_DMm = ((utc_incomming[2] - '0')*10) + (utc_incomming[3] - '0');
	minute_DMm += ((utc_incomming[5] - '0')*0.1) + ((utc_incomming[6] - '0')*0.01) + ((utc_incomming[7] - '0')*0.001) + ((utc_incomming[8] - '0')*0.0001);

	gps_struct->latitude_deg_s = (degree_DMm + (minute_DMm/60));
}

void gps_ProcessLongitude(uint8_t utc_incomming[],GPS_t *gps_struct){

	int8_t degree_DMm = 0;
	float minute_DMm = 0;

	/*Add reader degree*/
	degree_DMm = ((utc_incomming[1] - '0')*10) + (utc_incomming[2] - '0');
	/*Add first part of minutes*/
	minute_DMm = ((utc_incomming[3] - '0')*10) + (utc_incomming[4] - '0');
	minute_DMm += ((utc_incomming[5] - '0')*0.1) + ((utc_incomming[6] - '0')*0.01) + ((utc_incomming[7] - '0')*0.001) + ((utc_incomming[8] - '0')*0.0001);

	gps_struct->longitude_deg_s = (degree_DMm + (minute_DMm/60));
}

void gps_ProcessNorthSouth(uint8_t utc_incomming[],GPS_t *gps_struct){

	if(utc_incomming[0] == 'N'){
		gps_struct->north_south = north;
	}
	else{
		gps_struct->north_south = south;
	}
}

void gps_ProcessEastWest(uint8_t utc_incomming[],GPS_t *gps_struct){

	if(utc_incomming[0] == 'E'){
		gps_struct->east_west = east;
	}
	else{
		gps_struct->east_west = west;
	}
}

void gps_ProcessFix(uint8_t utc_incomming[],GPS_t *gps_struct){

	if(utc_incomming[0] == '1'){
		gps_struct->qualification = fix_gps;
	}
	else{
		if(utc_incomming[0] == '2'){
			gps_struct->qualification = fix_dgps;
		}
		else{
			gps_struct->qualification = unvalid;
		}
	}
}

void gps_ProcessSatelliteCount(uint8_t utc_incomming[],GPS_t *gps_struct){

	gps_struct->satelite_number = ((utc_incomming[0] - '0')*10) + (utc_incomming[1] - '0');
}

void gps_ProcessAltitude(uint8_t utc_incomming[],GPS_t *gps_struct){

	float altitude = 0;
	uint8_t dot_find = 0;
	for(int i=0;i<sizeof(utc_incomming);i++){

		if(utc_incomming[i] == '.'){
			dot_find = 1;
			i++;
		}

		if(dot_find){
			altitude +=  (float)((utc_incomming[i] - '0')*0.1);
			break;

		}
		else{
			altitude = (altitude*10) + (utc_incomming[i] - '0');
		}

	}
	gps_struct->altitude_deg_s = altitude;
}

void gps_ProcessAltitudeCorre(uint8_t utc_incomming[],GPS_t *gps_struct){

	float altitude = 0;
	uint8_t dot_find = 0;
	for(int i=0;i<sizeof(utc_incomming);i++){

		if(utc_incomming[i] == '.'){
			dot_find = 1;
			i++;
		}

		if(dot_find){
			altitude +=  (float)((utc_incomming[i] - '0')*0.1);
			break;

		}
		else{
			altitude = (altitude*10) + (utc_incomming[i] - '0');
		}

	}
	gps_struct->altitude_correction = altitude;
}
