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
#define DO_STEP_X LL_TIM_EnableCounter(TIM16)
#define DO_STEP_Y LL_TIM_EnableCounter(TIM17)

#define SET_DIR_POSITIVE_X LL_GPIO_ResetOutputPin(X_DIR_GPIO_Port, X_DIR_Pin)
#define SET_DIR_NEGATIVE_X LL_GPIO_SetOutputPin(X_DIR_GPIO_Port, X_DIR_Pin)
#define SET_DIR_POSITIVE_Y LL_GPIO_SetOutputPin(Z_DIR_GPIO_Port, Z_DIR_Pin)
#define SET_DIR_NEGATIVE_Y LL_GPIO_ResetOutputPin(Z_DIR_GPIO_Port, Z_DIR_Pin)

#define ENABLE_MOTOR_X LL_GPIO_ResetOutputPin(X_ENABLE_GPIO_Port, X_ENABLE_Pin)
#define DISABLE_MOTOR_X LL_GPIO_SetOutputPin(X_ENABLE_GPIO_Port, X_ENABLE_Pin)
#define ENABLE_MOTOR_Y LL_GPIO_ResetOutputPin(Z_ENABLE_GPIO_Port, Z_ENABLE_Pin)
#define DISABLE_MOTOR_Y LL_GPIO_SetOutputPin(Z_ENABLE_GPIO_Port, Z_ENABLE_Pin)

#define IS_HOMED (homed_x && homed_y)

#define ENABLE_ENDSTOP_X LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_7);
#define DISABLE_ENDSTOP_X LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_7);
#define ENABLE_ENDSTOP_Y LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_15);
#define DISABLE_ENDSTOP_Y LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_15);


// Definitions:

#define HOMING_STEPS_X 750
#define HOMING_STEPS_Y 700


void handle_endstop_x();
void handle_endstop_y();
void do_home();


#endif /* INC_MOTORS_H_ */
