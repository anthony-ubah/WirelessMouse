/**
  ******************************************************************************
  * File Name					: kalman_filter.h
	*	Description				: header for kalmam filter
	*	Author						: Stephan Greto-McGrath, Arthur Fabre
	*	Date							: Oct 4th, 2016
  ******************************************************************************
  */

#include "arm_math.h"
#ifndef _KALMAN_FILTER
#define _KALMAN_FILTER
	// define kalman state
	
typedef struct data{
	float raw;
	float filtered;
}data;

// used to store states in calling function
typedef struct kalman_state{
	arm_matrix_instance_f32 f;
	arm_matrix_instance_f32 h;
	arm_matrix_instance_f32 q;
	arm_matrix_instance_f32 r;
	arm_matrix_instance_f32 x;
	arm_matrix_instance_f32 p;
	arm_matrix_instance_f32 k;
}kalman_state;
int kalman_filter(float* InputArray, float* OutputArray, struct kalman_state* kstate, int Length, int State_dimension, int Measurement_dimension);
#endif
