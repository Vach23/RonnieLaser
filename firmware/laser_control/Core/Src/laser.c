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
#define RX_BUFF_LEN 8

#pragma pack(1)

union {
	char rx_buff[RX_BUFF_LEN];
	struct {
		char name;
		int16_t coord_x;
		int16_t coord_y;
		int16_t speed;
		char newline;
	} rx_data;
} rx_message;


char rx_buffer[RX_BUFF_LEN];

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
	/*
	do_home(true);

	srand(4564);
	if (IS_HOMED) {
		LL_TIM_EnableCounter(TIM2);
		LL_GPIO_SetOutputPin(LASER_ENABLE_GPIO_Port, LASER_ENABLE_Pin);
	}
	*/

	  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, (uint8_t *) rx_buffer, RX_BUFF_LEN);
	  __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
}

void my_loop() {
	  if (rx_ready) {
		  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, (uint8_t *) rx_buffer, RX_BUFF_LEN);
		  __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
		  if (rx_buffer[7] == '\n'){
			  memcpy(rx_message.rx_buff, rx_buffer, RX_BUFF_LEN);
			  process_rx_data();
		  }
		  rx_ready = false;
	  }
}

void process_rx_data(){
	switch (rx_message.rx_data.name) {
	case 'L':
		if (rx_message.rx_data.coord_x == 0) {
			LL_GPIO_SetOutputPin(LASER_ENABLE_GPIO_Port, LASER_ENABLE_Pin);
			break;
		}
		LL_GPIO_ResetOutputPin(LASER_ENABLE_GPIO_Port, LASER_ENABLE_Pin);
		break;
	case 'H':
		do_home(true);
		LL_TIM_EnableCounter(TIM2);
		break;
	case 'G':
		new_steps_x = rx_message.rx_data.coord_x;
		new_steps_y = rx_message.rx_data.coord_y;
		new_coordinates = true;
		break;
	case 'S':
		new_steps_x = rx_message.rx_data.coord_x;
		new_steps_y = rx_message.rx_data.coord_y;
		speed = rx_message.rx_data.speed;
		speed2 = (int)((float)speed*1.414f);
		new_coordinates = true;
	default:
		break;
	}
}


