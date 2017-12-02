/**
  ******************************************************************************
	*	File Name					: display.h
	*	Description				: header file for display.c
	*	Author						: Stephan Greto-McGrath, Arthur Fabre
	*	Date							: Oct 4th, 2016
  ******************************************************************************
  */
#ifndef _DISPLAY
#define _DISPLAY
#include <cmsis_os.h>
// segment selection
#define PIN_A GPIO_PIN_4 
#define PIN_B GPIO_PIN_5
#define PIN_C GPIO_PIN_6
#define PIN_D GPIO_PIN_7
#define PIN_E GPIO_PIN_8
#define PIN_F GPIO_PIN_9
#define PIN_G GPIO_PIN_10
// digit selection
#define PIN_DIG_1 GPIO_PIN_11 
#define PIN_DIG_2 GPIO_PIN_12
#define PIN_DIG_3 GPIO_PIN_13
#define PIN_DIG_4 GPIO_PIN_14
// decimal place
#define PIN_DEC GPIO_PIN_15
// make life easy
#define SEGMENT_PINS PIN_A|PIN_B|PIN_C|PIN_D|PIN_E|PIN_F|PIN_G|PIN_DEC
#define LINE_PINS PIN_DIG_1|PIN_DIG_2|PIN_DIG_3|PIN_DIG_4
#define ALL_PINS PIN_A|PIN_B|PIN_C|PIN_D|PIN_E|PIN_F|PIN_G|PIN_DEC|PIN_DIG_1|PIN_DIG_2|PIN_DIG_3|PIN_DIG_4
// gpio bank and clock
#define DISPLAY_PORT GPIOE		
#define DISPLAY_PORT_CLK_EN __HAL_RCC_GPIOE_CLK_ENABLE();
// numbers as combinations of segments
#define SEG_0 PIN_A|PIN_B|PIN_C|PIN_D|PIN_E|PIN_F
#define SEG_1 PIN_B|PIN_C
#define SEG_2 PIN_A|PIN_B|PIN_D|PIN_E|PIN_G
#define SEG_3 PIN_A|PIN_B|PIN_C|PIN_D|PIN_G
#define SEG_4 PIN_B|PIN_C|PIN_F|PIN_G
#define SEG_5 PIN_A|PIN_C|PIN_D|PIN_F|PIN_G
#define SEG_6 PIN_A|PIN_C|PIN_D|PIN_E|PIN_F|PIN_G
#define SEG_7 PIN_A|PIN_B|PIN_C
#define SEG_8 PIN_A|PIN_B|PIN_C|PIN_D|PIN_E|PIN_F|PIN_G
#define SEG_9 PIN_A|PIN_B|PIN_C|PIN_D|PIN_G|PIN_F
// letters as combinations of segments
#define SEG_A PIN_A|PIN_B|PIN_C|PIN_E|PIN_F|PIN_G
#define SEG_B PIN_C|PIN_D|PIN_E|PIN_F|PIN_G
#define SEG_C PIN_A|PIN_D|PIN_E|PIN_F
#define SEG_D PIN_B|PIN_C|PIN_D|PIN_E|PIN_G
#define SEG_E PIN_A|PIN_D|PIN_E|PIN_F|PIN_G
#define SEG_F PIN_A|PIN_E|PIN_F|PIN_G
#define SEG_G PIN_A|PIN_B|PIN_C|PIN_D|PIN_G
#define SEG_H PIN_B|PIN_C|PIN_E|PIN_F|PIN_G
#define SEG_I PIN_E|PIN_F
#define SEG_J	PIN_B|PIN_C|PIN_D|PIN_E
#define SEG_K PIN_B|PIN_C|PIN_E|PIN_F|PIN_G
#define SEG_L PIN_D|PIN_E|PIN_F
#define SEG_M PIN_A|PIN_C|PIN_E
#define SEG_N PIN_C|PIN_E|PIN_G
#define SEG_O	PIN_A|PIN_B|PIN_C|PIN_D|PIN_E|PIN_F
#define SEG_P	PIN_A|PIN_B|PIN_E|PIN_F|PIN_G
#define SEG_Q	PIN_A|PIN_B|PIN_C|PIN_G
#define SEG_R PIN_E|PIN_G
#define SEG_S PIN_A|PIN_C|PIN_D|PIN_F|PIN_G
#define SEG_T	PIN_D|PIN_E|PIN_F|PIN_G
#define SEG_U	PIN_B|PIN_C|PIN_D|PIN_E|PIN_F
#define SEG_V PIN_C|PIN_D|PIN_E
#define SEG_W PIN_B|PIN_D|PIN_F
#define SEG_X	PIN_B|PIN_C|PIN_E|PIN_F|PIN_G
#define SEG_Y	PIN_B|PIN_C|PIN_D|PIN_F|PIN_G
#define SEG_Z PIN_A|PIN_B|PIN_D|PIN_E|PIN_G
// functions
void display_init(void);
void start_display_thread(void *args);
void display_thread(void const *args);
void set_display(char display_set[], char flag);
extern osThreadId display_thread_ID;
#endif
