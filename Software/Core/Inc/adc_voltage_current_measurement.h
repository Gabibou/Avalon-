/*
 * adc_voltage_current_measurement.h
 *
 *  Created on: Feb 14, 2023
 *      Author: Bertet Gabriel
 */

//This file is use to define every needed value and function used by ADC or ADC measurement
#include <stdint.h>

#ifndef INC_ADC_VOLTAGE_CURRENT_MEASUREMENT_H_
#define INC_ADC_VOLTAGE_CURRENT_MEASUREMENT_H_

//Value of the shunt resistor(used for current measurement on 5V and 3.3V)
#define SHUNT_RESISTOR_OHM 1
//Voltage reference applied to the ADC
#define VREF_ADC 3.33
//ADC RES 12bit = 4096 10bit =1023 ...
#define ADC_RESOLUTION 4096
//current measurement divider
#define CURRENT_MEASUREMENT_DIVIDER 2
//battery monitoring voltage divider --> (R1+R2)/R2
#define VOLTAGE_DIVIDER_BATTERY_MONITORING 4.031
//adc number of channel
#define ADC_CHANNEL_NUMBER 3
//ADC dma order (rank0 mean his value his 0 ...)
#define ADC_CURRENT0_RANK 0	//first of the conversion transfert via DMA
#define ADC_CURRENT1_RANK 1
#define ADC_VOLTAGE_RANK 2

typedef struct{
	float BatteryVoltage;
	float CurrentConsumption;
}POWER_t;



//Function prototype
void CURRENT_Measurement_Calculation(POWER_t *PowerClass,uint32_t adc_value_1,uint32_t adc_value_2);

void Battery_Monitoring_Calculation(POWER_t *PowerClass,uint32_t adc_value_3);

#endif /* INC_ADC_VOLTAGE_CURRENT_MEASUREMENT_H_ */
