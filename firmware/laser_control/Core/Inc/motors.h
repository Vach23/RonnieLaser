/*
 * motors.h
 *
 *  Created on: Dec 11, 2022
 *      Author: Uzivatel
 */

#ifndef INC_MOTORS_H_
#define INC_MOTORS_H_

#include <stdbool.h>
#include "main.h"

// MACROS:
#define ENABLE_MOTOR_X LL_GPIO_ResetOutputPin(X_ENABLE_GPIO_Port, X_ENABLE_Pin)
#define DISABLE_MOTOR_X LL_GPIO_SetOutputPin(X_ENABLE_GPIO_Port, X_ENABLE_Pin)
#define ENABLE_MOTOR_Y LL_GPIO_ResetOutputPin(Z_ENABLE_GPIO_Port, Z_ENABLE_Pin)
#define DISABLE_MOTOR_Y LL_GPIO_SetOutputPin(Z_ENABLE_GPIO_Port, Z_ENABLE_Pin)

#define ENABLE_ENDSTOP_X LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_7);
#define DISABLE_ENDSTOP_X LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_7);
#define ENABLE_ENDSTOP_Y LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_15);
#define DISABLE_ENDSTOP_Y LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_15);

#define IS_HOMED (homed_x && homed_y)

#define ENABLE_MOTION_CONTROL LL_TIM_EnableCounter(TIM2);
#define DISABLE_MOTION_CONTROL LL_TIM_DisableCounter(TIM2);

// Definitions:
#define HOMING_STEPS_X 750
#define HOMING_STEPS_Y 700

extern bool homed_x;
extern bool homed_y;

extern int _steps_x;
extern int _steps_y;
extern int new_steps_x;
extern int new_steps_y;
extern int _dir_x;
extern int _dir_y;
extern bool new_coordinates;
extern bool finished;

extern int speed;
extern int speed2;

void set_dir_positive_x();
void set_dir_negative_x();
void set_dir_positive_y();
void set_dir_negative_y();

void handle_endstop_x();
void handle_endstop_y();
void do_step_x();
void do_step_y();

void do_home(bool);
#endif /* INC_MOTORS_H_ */
