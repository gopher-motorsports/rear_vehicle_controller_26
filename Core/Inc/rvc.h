#ifndef INC_steering_wheel_module_26_H
#define INC_steering_wheel_module_26_H

#include "main.h"
#include "GopherCAN.h"
#include <stdio.h>

// ================================== READY TO DRIVE PARAMETERS =================================
#define PREDRIVE_BRAKE_THRESH_psi  10  // The minimum brake pressure to enter the driving state
#define PREDRIVE_BUTTON_PRESSED    1    // The value of the button parameter when pressed
#define PREDRIVE_TIME_ms           2000 // The length of predrive in ms
#define RTD_BUTTON_PUSHED          (GPIO_PIN_RESET)
#define TS_ON_THRESHOLD_VOLTAGE_V  190
#define TSSI_FLASH_PERIOD_ms       300
#define TSSI_RESET_TIME_ms         12000
// ==============================================================================================


// =========================================Current Sensing=====================================================
#define CURRENT_HIGH_RAIL_THRESHOLD             852
#define CURRENT_LOW_RAIL_THRESHOLD              87
#define CURRENT_LOW_TO_HIGH_SWITCH_THRESHOLD    75
// The size in Amps across which current from channel 1 is blended with channel 2
#define CHANNEL_FILTERING_WIDTH 4
// ===========================================LED Timing========================================================
#define TSSI_RED_BLINK_TIME_ms  333 //corresponds to 3 Hz

// =============================================================================================================

typedef enum
{
	UNINITIALIZED = 0,	// Value on startup
	WORKING,			// Data nominal
	FAULTING			// Unavailable data or hardware issue
} Sensor_Status_E;


void init(CAN_HandleTypeDef* hcan_ptr);
void can_buffer_handling_loop();
void main_loop();
void update_TSSI_LED();
float getTractiveSystemCurrent();
#endif /* INC_steering_wheel_module_26_H */