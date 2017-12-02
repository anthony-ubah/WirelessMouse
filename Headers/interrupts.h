/**
  ******************************************************************************
  * File Name					: interrupts.h
	*	Description				: header for interrupts, externs timing flags
	*	Author						: Stephan Greto-McGrath, Arthur Fabre
	*	Date							: Oct 28th, 2016
  ******************************************************************************
  */
#ifndef _INTERRUPTS
#define _INTERRUPTS
extern volatile int display_time_elapsed; // TIM2
extern volatile int keypad_time_elapsed; // TIM3
extern volatile int temp_time_elapsed; // TIM4
extern volatile int control_time_elapsed; // TIM2
extern volatile int mouse_time_elapsed; // TIM2
void timers_init(int init);
#endif
