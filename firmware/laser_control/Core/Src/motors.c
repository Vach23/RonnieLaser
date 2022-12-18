/*
 * motors.c
 *
 *  Created on: Dec 11, 2022
 *      Author: Uzivatel
 */

#include "main.h"
#include "motors.h"

bool homed_x = false;
bool homed_y = false;
bool is_homing = false;

int _steps_x = 0;
int _steps_y = 0;
int new_steps_x = 0;
int new_steps_y = 0;
int _dir_x = 1;
int _dir_y = 1;
bool new_coordinates = false;
bool finished = true;

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



inline void do_step_x() {
	LL_TIM_EnableCounter(TIM16);
	_steps_x += _dir_x;
}

inline void do_step_y() {
	LL_TIM_EnableCounter(TIM17);
	_steps_y += _dir_y;
}

inline void set_dir_positive_x() {
	LL_GPIO_ResetOutputPin(X_DIR_GPIO_Port, X_DIR_Pin);
	_dir_x = 1;

}

inline void set_dir_negative_x() {
	LL_GPIO_SetOutputPin(X_DIR_GPIO_Port, X_DIR_Pin);
	_dir_x = -1;
}

inline void set_dir_positive_y() {
	LL_GPIO_SetOutputPin(Z_DIR_GPIO_Port, Z_DIR_Pin);
	_dir_y = 1;
}

inline void set_dir_negative_y() {
	LL_GPIO_ResetOutputPin(Z_DIR_GPIO_Port, Z_DIR_Pin);
	_dir_y = -1;
}

void do_home_x() {
	ENABLE_MOTOR_X;
	ENABLE_ENDSTOP_X;
	set_dir_positive_x();
	while (!(homed_x)) {
		do_step_x();
		LL_mDelay(1);
	}

	set_dir_negative_x();
	for (int i = 0; i < HOMING_STEPS_X; i++) {
		do_step_x();
		LL_mDelay(1);
	}
}

void do_home_y() {
	ENABLE_MOTOR_Y;
	ENABLE_ENDSTOP_Y;
	set_dir_positive_y();
	while (!(homed_y)) {
		do_step_y();
		LL_mDelay(1);
	}

	set_dir_negative_y();
	for (int i = 0; i < HOMING_STEPS_Y; i++) {
		do_step_y();
		LL_mDelay(1);
	}
}

void disable_motors() {
	DISABLE_MOTOR_X;
	DISABLE_MOTOR_Y;
}


void do_steps_x(uint32_t steps, uint32_t ms_delay) {
	for (uint32_t i = 0; i < steps; i++) {
		do_step_x();
		LL_mDelay(ms_delay);
	}
}

void do_steps_y(uint32_t steps, uint32_t ms_delay) {
	for (uint32_t i = 0; i < steps; i++) {
		do_step_y();
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
	set_dir_negative_y();
	ENABLE_MOTOR_Y;
	do_steps_y(20, 1);
	do_home_y();
	// Potom motor X
	do_home_x();

	is_homing = false;
	disable_motors();

	_steps_x = 0;
	_steps_y = 0;
}


