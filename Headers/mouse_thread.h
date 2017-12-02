////////////////////////////////////////////////////////////////////////////////
//	File Name					: mouse_thread.h
//	Description				: Header file for mouse thread
//	Author						: Harsh Aurora
//	Date							: Nov 8, 2016
////////////////////////////////////////////////////////////////////////////////

#ifndef _MOUSE_THREAD
#define _MOUSE_THREAD
#include <stdint.h>
#include <cmsis_os.h>
#include <stm32f4xx_hal.h>
#include <rl_usb.h>
#include "interrupts.h"
#include "supporting_functions.h"
#include "acc.h"

#define BUTTON_1 GPIO_PIN_0
#define BUTTON_2 GPIO_PIN_1
#define BUTTON_3 GPIO_PIN_4
#define CLICK_BANK GPIOB
#define CLICK_BANK_CLK_EN __HAL_RCC_GPIOB_CLK_ENABLE
#define BUTTONS BUTTON_1|BUTTON_2|BUTTON_3

extern uint8_t  mouse_in_report[4];
extern osThreadId mouse_thread_ID;

//		Exported Functios		//
void start_mouse_thread(void *args);
void mouse_init(int func);

#endif
