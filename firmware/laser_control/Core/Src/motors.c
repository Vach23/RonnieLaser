/*
 * motors.c
 *
 *  Created on: Dec 11, 2022
 *      Author: Uzivatel
 */

#include "main.h"
#include <motors.h>

bool homed_x = false;
bool homed_y = false;
bool is_homing = false;

inline void handle_endstop_x() {
	if (is_homing) {
		DISABLE_ENDSTOP_X;
		homed_x = true;
	}
}

inline void handle_endstop_y() {
	//DO_STEP_Y;
	if (is_homing) {
		DISABLE_ENDSTOP_Y;
		homed_y = true;
	}
}

void do_home_x() {
	ENABLE_MOTOR_X;
	ENABLE_ENDSTOP_X;
	SET_DIR_POSITIVE_X;
	while (!(homed_x)) {
		//DO_STEP_Y;
		DO_STEP_X;
		LL_mDelay(1);
	}

	SET_DIR_NEGATIVE_X;
	for (int i = 0; i < HOMING_STEPS_X; i++) {
		DO_STEP_X;
		LL_mDelay(1);
	}
}

void do_home_y() {
	ENABLE_MOTOR_Y;
	ENABLE_ENDSTOP_Y;
	SET_DIR_POSITIVE_Y;
	while (!(homed_y)) {
		//DO_STEP_Y;
		DO_STEP_Y;
		LL_mDelay(1);
	}

	SET_DIR_NEGATIVE_Y;
	for (int i = 0; i < HOMING_STEPS_Y; i++) {
		DO_STEP_Y;
		LL_mDelay(1);
	}
}

void disable_motors() {
	DISABLE_MOTOR_X;
	DISABLE_MOTOR_Y;
}


void do_steps_x(uint32_t steps, uint32_t ms_delay) {
	for (uint32_t i = 0; i < steps; i++) {
		DO_STEP_X;
		LL_mDelay(ms_delay);
	}
}

void do_steps_y(uint32_t steps, uint32_t ms_delay) {
	for (uint32_t i = 0; i < steps; i++) {
		DO_STEP_Y;
		LL_mDelay(ms_delay);
	}
}



void do_home() {
	homed_x = false;
	homed_y = false;
	is_homing = true;

	disable_motors();
	DISABLE_ENDSTOP_Y;
	DISABLE_ENDSTOP_X;
	// První allways Y:
	// Safety steps:
	SET_DIR_NEGATIVE_Y;
	ENABLE_MOTOR_Y;
	do_steps_y(20, 1);
	do_home_y();
	// Potom motor X
	do_home_x();

	is_homing = false;
	disable_motors();
}
