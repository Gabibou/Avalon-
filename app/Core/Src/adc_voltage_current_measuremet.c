/*
 * adc_voltage_current_measuremet.c
 *
 *  Created on: Feb 14, 2023
 *      Author: 33768
 */

#include "adc_voltage_current_measurement.h"



void Battery_ReadBatteryVoltage(Battery_t * battery_struct,uint16_t BatteryMonitoringData[]){
	battery_struct->BatteryVoltage = (BatteryMonitoringData[BATTERY_VOLTAGE_RANK]*ADC_VOLTAGE_REF)/ADC_BIT_VALUE;
}

void Battery_ReadCurrent3V3(Battery_t * battery_struct,uint16_t BatteryMonitoringData[]){
	float32_t shunt_resistor_voltage = (BatteryMonitoringData[BATTERY_3V3CURR_RANK]*ADC_VOLTAGE_REF)/ADC_BIT_VALUE;
	battery_struct->CurrentConsumption3V3 = shunt_resistor_voltage/SHUNT_RESISTOR_VALUE;
}

void Battery_ReadCurrent5V(Battery_t * battery_struct,uint16_t BatteryMonitoringData[]){
	float32_t shunt_resistor_voltage = (BatteryMonitoringData[BATTERY_5VCURR_RANK]*ADC_VOLTAGE_REF)/ADC_BIT_VALUE;
	battery_struct->CurrentConsumption5V = shunt_resistor_voltage/SHUNT_RESISTOR_VALUE;
}

void Battery_RemaningTime(Battery_t * battery_struct){

	float32_t current_average = battery_struct->CurrentConsumption3V3 + battery_struct->CurrentConsumption5V;
	float32_t battery_remaining_hour = BATTERRY_CAPACITY/current_average;
	battery_struct->battery_left_ms = (battery_remaining_hour*3600000);
}
