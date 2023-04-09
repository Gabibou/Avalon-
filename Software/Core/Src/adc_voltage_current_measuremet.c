/*
 * adc_voltage_current_measuremet.c
 *
 *  Created on: Feb 14, 2023
 *      Author: 33768
 */

#include "adc_voltage_current_measurement.h"








//------------------------------------------------ FUNCTION ------------------------------------------------
void CURRENT_Measurement_Calculation(POWER_t *PowerClass,uint32_t adc_value_1,uint32_t adc_value_2){

	//quantum of the adc1
	float quantum_adc1 = VREF_ADC/ADC_RESOLUTION;
	float voltage1 = (quantum_adc1*adc_value_1)*CURRENT_MEASUREMENT_DIVIDER;
	float voltage2 = (quantum_adc1*adc_value_2)*CURRENT_MEASUREMENT_DIVIDER;
	float current = (voltage1 - voltage2)/SHUNT_RESISTOR_OHM;
	if(current>0){
		PowerClass->CurrentConsumption = current;
	}
}

void Battery_Monitoring_Calculation(POWER_t *PowerClass,uint32_t adc_value3){
	float quantum_adc1 = VREF_ADC/ADC_RESOLUTION;
	PowerClass->BatteryVoltage = (adc_value3*quantum_adc1)*VOLTAGE_DIVIDER_BATTERY_MONITORING;
}

