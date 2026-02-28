#ifndef INC_steering_wheel_module_26_H
#define INC_steering_wheel_module_26_H

#include "main.h"
#include "GopherCAN.h"
#include "cooling.h"
#include <stdio.h>

// ====================================== PULLUP PARAMETERS =====================================
#define RAD_FAN_ON (GPIO_PIN_SET)
#define RAD_FAN_OFF (GPIO_PIN_RESET)
#define PUMP_DIGITAL_ON (GPIO_PIN_SET)
#define PUMP_DIGITAL_OFF (GPIO_PIN_RESET)
// ==============================================================================================


// ====================================== COOLING PARAMETERS ====================================
#define INVERTER_PUMP_POWER_ON_THRESH 45.0f //Low Threshold Turn On
#define MOTOR_PUMP_THRESH_C         45.0f // Motor temperature at which the cooling system turns on

#define INVERTER_FAN_THRESH_C    50.0f  // Inverter temperature at which the cooling system turns on
#define MOTOR_FAN_THRESH_C       50.0f  // Motor temperature at which the cooling system turns on
#define COOLING_HYSTERESIS_C     5.0f   // Hysteresis when confined to digital signal (on/off)
#define CAR_SPEED_FAN_HYS	     5.0f   // Hysteresis on Car speed for turning on/off fans
#define CAR_SPEED_FAN_THRESH	 20.0f  // Car speed at which air cooling from movement is enough
//#define USING_PUMP_PWM
#define PUMP_OFF            0  //0% duty cycle --> 0/49999
#define PUMP_50_PERCENT     25000 //50% duty cycle --> 25000/49999
#define PUMP_100_PERCENT    49999 //100% duty cycle --> 49999/49999
#define PUMP_COUNTER_PERIOD	49999
#define PUMP_PERCENT_OFFSET 0.5f //if the pump is on it will statr at 50%
// ==============================================================================================

typedef enum {
    PUMP_STATE_OFF,
    PUMP_STATE_ON,
} PUMP_STATE;

typedef enum {
    FAN_STATE_OFF,
    FAN_STATE_ON
} FAN_STATE;


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
void update_cooling();     // Controls/updates the cooling system
void init_Pump(TIM_HandleTypeDef* timer_address, U32 channel);

#endif /* INC_steering_wheel_module_26_H */