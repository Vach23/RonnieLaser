/*
 * app.c
 *
 *  Created on: 6. 12. 2022
 *      Author: vaclav.nemec
 */

#include "main.h"
#include <laser.h>
#include <motors.h>


void my_init() {
	SET_DIR_POSITIVE_X;
	SET_DIR_POSITIVE_Y;

	DISABLE_MOTOR_X;
	DISABLE_MOTOR_Y;

	// MOTOR_X
	LL_TIM_CC_EnableChannel(TIM16, LL_TIM_CHANNEL_CH1);
	LL_TIM_EnableAllOutputs(TIM16);

	// MOTOR_Y
	LL_TIM_CC_EnableChannel(TIM17, LL_TIM_CHANNEL_CH1);
	LL_TIM_EnableAllOutputs(TIM17);

	ENABLE_MOTOR_Y;
	LL_TIM_SetCounter(TIM2, 0);
	LL_TIM_EnableIT_CC1(TIM2);
	LL_TIM_EnableCounter(TIM2);


	//test();
	//do_home();


}

void my_loop() {

}


