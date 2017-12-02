/**
  ******************************************************************************
  * File Name					: keypad.h
	*	Description				: header for keypad file
	*	Author						: Stephan Greto-McGrath, Arthur Fabre
	*	Date							: Oct 4th, 2016
  ******************************************************************************
  */
#ifndef _KEYPAD
#define _KEYPAD
#include <cmsis_os.h>
#define ROW_1 GPIO_PIN_0
#define ROW_2 GPIO_PIN_1
#define ROW_3 GPIO_PIN_2
#define ROW_4 GPIO_PIN_3
#define COL_1 GPIO_PIN_6
#define COL_2 GPIO_PIN_7
#define COL_3 GPIO_PIN_8 
#define COL_4 GPIO_PIN_9
#define ROW_KEYS ROW_1|ROW_2|ROW_3|ROW_4
#define COL_KEYS COL_1|COL_2|COL_3|COL_4
#define KEY_PORT GPIOD
#define KEY_PORT_CLK_EN __HAL_RCC_GPIOD_CLK_ENABLE();


void keypad_init(void);
void keypad_thread(void const *args);
void start_keypad_thread(void *args);
char get_key(void);

extern osThreadId keypad_thread_ID;
#endif
