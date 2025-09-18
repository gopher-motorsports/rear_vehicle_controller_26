#ifndef INC_steering_wheel_module_26_H
#define INC_steering_wheel_module_26_H

#include "main.h"
#include "GopherCAN.h"
#include <stdio.h>
#include "stm32f4xx_hal_can.h"

void init(CAN_HandleTypeDef* hcan_ptr);
void can_buffer_handling_loop();
void main_loop();

#endif /* INC_steering_wheel_module_26_H */