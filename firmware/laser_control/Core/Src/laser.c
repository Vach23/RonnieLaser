/*
 * app.c
 *
 *  Created on: 6. 12. 2022
 *      Author: vaclav.nemec
 */

#include "main.h"
#include <laser.h>
#include <motors.h>
#include <stdlib.h>
#include <math.h>

#define R 300

void my_init() {
	set_dir_positive_x();
	set_dir_positive_y();

	DISABLE_MOTOR_X;
	DISABLE_MOTOR_Y;

	// MOTOR_X
	LL_TIM_CC_EnableChannel(TIM16, LL_TIM_CHANNEL_CH1);
	LL_TIM_EnableAllOutputs(TIM16);

	// MOTOR_Y
	LL_TIM_CC_EnableChannel(TIM17, LL_TIM_CHANNEL_CH1);
	LL_TIM_EnableAllOutputs(TIM17);

	//ENABLE_MOTOR_Y;
	LL_TIM_SetCounter(TIM2, 0);
	LL_TIM_EnableIT_CC1(TIM2);
	//LL_TIM_EnableCounter(TIM2);


	//test();
	do_home(true);

	srand(4564);
	if (IS_HOMED) {
		LL_TIM_EnableCounter(TIM2);
		LL_GPIO_SetOutputPin(LASER_ENABLE_GPIO_Port, LASER_ENABLE_Pin);
	}
}

void my_loop() {
	/*
	static float deg = 0.0;
	deg+=0.1;
	new_steps_x = (int)(sin(deg)*R);
	new_steps_y = (int)(cos(deg)*R);
	*/


	new_steps_x = rand()%1000-500;
	new_steps_y = rand()%1000-500;
	speed = rand()%50+10;
	speed2 = (int)((float)speed*1.4f);

	new_coordinates = true;
	while (!finished) {
		LL_mDelay(1);
	}
}


