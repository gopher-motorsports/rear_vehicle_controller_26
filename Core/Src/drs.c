// TODO: Port over RVC 2025 Code
// Fortify the DRS shutoff conditons reached to include both steering angle & brake
/*
 * drs.c
 *
 *  Created on: Mar 21, 2024
 *      Author: chris
 */
#include "drs.h"
#include "main.h"
#include "rvc.h"

TIM_HandleTypeDef* DRS_Timer;
U32 DRS_Channel;
U32 last_button_press_time = 0;	//for implementing toggle functionality
U8 drs_button_state;
bool turning_wheel_hysteresis_on = false;

//local function prototypes
bool drs_shutoff_conditions_reached();

//DRS brake thresholds
typedef enum {
	DRS_BRAKE_NORMAL = 0,
	DRS_BRAKE_TRIPPED = 1
} DRS_BRAKE_STATES;

void init_DRS_servo(TIM_HandleTypeDef* timer_address, U32 channel){
	DRS_Timer = timer_address;
	DRS_Channel = channel;
	HAL_TIM_PWM_Start(DRS_Timer, DRS_Channel); //turn on PWM generation
}

void set_DRS_Servo_Position_Open(bool set_to_open){
	if(set_to_open == false){
		__HAL_TIM_SET_COMPARE(DRS_Timer, DRS_Channel, CLOSED_POS);
	}
	else{
		__HAL_TIM_SET_COMPARE(DRS_Timer, DRS_Channel, OPEN_POS);
	}
}

void set_DRS_Servo_Position_Button(){
	//hold down to keep DRS on
	drs_button_state = swButon2_state.data;

	if(drs_button_state == 1){
		__HAL_TIM_SET_COMPARE(DRS_Timer, DRS_Channel, OPEN_POS);
	}
	else{
		__HAL_TIM_SET_COMPARE(DRS_Timer, DRS_Channel, CLOSED_POS);
	}
}

void set_DRS_Servo_Position_Auto(){
	//Sets DRS position based on brake pressure and steering angle conditions
	if(drs_shutoff_conditions_reached()){
		__HAL_TIM_SET_COMPARE(DRS_Timer, DRS_Channel, CLOSED_POS);
	}
	else{
		__HAL_TIM_SET_COMPARE(DRS_Timer, DRS_Channel, OPEN_POS);
	}
}

bool drs_shutoff_conditions_reached(){
	static DRS_BRAKE_STATES drs_brake_state = NORMAL_STATE;
	switch (drs_brake_state){
		case DRS_BRAKE_TRIPPED:
			//brakes are not on and steering wheel is centered --> closed to open
			if((rvcBrakePressureRear_psi.data <= BRAKE_SHUTOFF_THRESHOLD - DRS_HYSTERESIS) && 
			(fvcSteeringAngle_deg.data > STEERING_ANGLE_LEFT_RETURN && fvcSteeringAngle_deg.data < STEERING_ANGLE_RIGHT_RETURN)){
				drs_brake_state = DRS_BRAKE_NORMAL;
				return false; //DRS should be open
		    }			
			return true; //DRS should be closed
			break;
		case DRS_BRAKE_NORMAL:
		default:
			//have breached DRS braking or steering angle threshold --> open to closed
			if((rvcBrakePressureRear_psi.data > BRAKE_SHUTOFF_THRESHOLD + DRS_HYSTERESIS) ||
			(fvcSteeringAngle_deg.data < STEERING_ANGLE_LEFT_SHUTOFF || fvcSteeringAngle_deg.data > STEERING_ANGLE_RIGHT_SHUTOFF)){
				drs_brake_state = DRS_BRAKE_TRIPPED;
				return true; //DRS should be closed
			}

			return false; //DRS should be open
			break;
	}

	//reaches here we are in trouble
	return false;	//DRS should be open
}