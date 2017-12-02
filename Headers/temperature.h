/**
  ******************************************************************************
	*	File Name					: temperature.h
	*	Description				: header file for temperature.c
	*	Author						: Stephan Greto-McGrath, Arthur Fabre
	*	Date							: Nov 6th, 2016
  ******************************************************************************
  */
	

#ifndef _TEMPERATURE
#define _TEMPERATURE
#include <kalman_filter.h>
#include <cmsis_os.h>
#define POLL_TIMEOUT 10000
#define TEMP_ADC ADC1
#define TEMP_ADC_CHANNEL ADC_CHANNEL_16
#define TEMP_ADC_CLK_EN 	__HAL_RCC_ADC1_CLK_ENABLE()
void start_temp_thread(void *args);
void temp_thread(void const *args);
extern osThreadId temp_thread_ID;
void temp_adc_init(void);
float get_temp(void);
#endif
