/*
 * adc_voltage_current_measurement.h
 *
 *  Created on: Feb 14, 2023
 *      Author: Bertet Gabriel
 */

//This file is use to define every needed value and function used by ADC for battery measurement
#include <stdint.h>
#include "main.h"

#ifndef INC_ADC_VOLTAGE_CURRENT_MEASUREMENT_H_
#define INC_ADC_VOLTAGE_CURRENT_MEASUREMENT_H_

/*Number of ADC conversion to be done*/
#define CONVERSION_COUNT 3
/*define ADC reference for*/
#define ADC_VOLTAGE_REF 3.3
/*ADC use 12bit so 2^12 = 4096*/
#define ADC_BIT_VALUE 4096
/*shunt resistor value in ohm*/
#define SHUNT_RESISTOR_VALUE 0.002
/*Coeff of division before ADC input*/
#define VOLTAGE_DIVIDER_RESISTOR_COEF 4
/*define the rank of battery voltage*/
#define BATTERY_VOLTAGE_RANK 2
/*define the rank of battery voltage*/
#define BATTERY_3V3CURR_RANK 1
/*define the rank of battery voltage*/
#define BATTERY_5VCURR_RANK 0

/*Maximum/Minimum voltage for 2S lipo battery*/
#define BATTERY_2S_MAX	8.4
#define BATTERY_2S_MIN	6.4	//Could go a little lower
/*Maximum/Minimum voltage for 3S lipo battery*/
#define BATTERY_3S_MAX	12.6
#define BATTERY_3S_MIN	9.6	//Could go a little lower
/*define battery capacity in Ah*/
#define BATTERRY_CAPACITY 2.2

typedef struct{
	float32_t BatteryVoltage;
	float32_t CurrentConsumption3V3;
	float32_t CurrentConsumption5V;
	uint32_t battery_left_ms;
}Battery_t;

void Battery_ReadBatteryVoltage(Battery_t * battery_struct,uint16_t BatteryMonitoringData[]);
void Battery_ReadCurrent3V3(Battery_t * battery_struct,uint16_t BatteryMonitoringData[]);
void Battery_ReadCurrent5V(Battery_t * battery_struct,uint16_t BatteryMonitoringData[]);
void Battery_RemaningTime(Battery_t * battery_struct);



#endif /* INC_ADC_VOLTAGE_CURRENT_MEASUREMENT_H_ */
