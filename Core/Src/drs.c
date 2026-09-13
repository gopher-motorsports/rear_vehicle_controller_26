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

typedef enum {
	DRS_STEERING_NORMAL = 0,
	DRS_STEERING_LEFT_TRIPPED = 1,
	DRS_STEERING_RIGHT_TRIPPED = 2
} DRS_STEERING_STATES;

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

void set_DRS_Servo_Position_Toggle(){
	//press button to toggle DRS state
	drs_button_state = swButon2_state.data;
	bool curr_pos = TRUE;

	if(drs_button_state == 1){
		if(curr_pos){
			__HAL_TIM_SET_COMPARE(DRS_Timer, DRS_Channel, CLOSED_POS);
		}
		else{
			__HAL_TIM_SET_COMPARE(DRS_Timer, DRS_Channel, OPEN_POS);
		}
		curr_pos = !curr_pos;
	}
	update_and_queue_param_u8(&rvcDrsEnabled_state, curr_pos);
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
	static DRS_BRAKE_STATES drs_brake_state = DRS_BRAKE_NORMAL;
	static DRS_STEERING_STATES drs_steering_state = DRS_STEERING_NORMAL;
	switch (drs_brake_state){
		case DRS_BRAKE_TRIPPED:
			//brakes are not on and steering wheel is centered --> closed to open
			if(rvcBrakePressureRear_psi.data <= BRAKE_SHUTOFF_THRESHOLD - DRS_HYSTERESIS){
				drs_brake_state = DRS_BRAKE_NORMAL;
				//only returns false after going through steering state machine
		    }		
			else {
				return true; //DRS should be closed
			}	
			break;
		case DRS_BRAKE_NORMAL:
		default:
			//have breached DRS braking or steering angle threshold --> open to closed
			if(rvcBrakePressureRear_psi.data > BRAKE_SHUTOFF_THRESHOLD + DRS_HYSTERESIS){
				drs_brake_state = DRS_BRAKE_TRIPPED;
				return true; //DRS should be closed
			}
	}
	switch (drs_steering_state){
		case DRS_STEERING_LEFT_TRIPPED:
			//steering wheel is centered --> left limit breached to normal
			if(fvcSteeringAngle_deg.data > STEERING_ANGLE_LEFT_RETURN && fvcSteeringAngle_deg.data < STEERING_ANGLE_RIGHT_RETURN){
				drs_steering_state = DRS_STEERING_NORMAL;
				return false; //DRS should be open
		    }			
			return true; //DRS should be closed
			break;
		case DRS_STEERING_RIGHT_TRIPPED:
			//steering wheel is centered --> right limit breached to normal
			if(fvcSteeringAngle_deg.data > STEERING_ANGLE_LEFT_RETURN && fvcSteeringAngle_deg.data < STEERING_ANGLE_RIGHT_RETURN){
				drs_steering_state = DRS_STEERING_NORMAL;
				return false; //DRS should be open
		    }			
			return true; //DRS should be closed
			break;
		case DRS_STEERING_NORMAL:
		default:
			//have breached DRS steering angle threshold --> normal to breached
			if(fvcSteeringAngle_deg.data < STEERING_ANGLE_LEFT_SHUTOFF || fvcSteeringAngle_deg.data > STEERING_ANGLE_RIGHT_SHUTOFF){
				if(fvcSteeringAngle_deg.data < STEERING_ANGLE_LEFT_SHUTOFF){
					drs_steering_state = DRS_STEERING_LEFT_TRIPPED;
				}
				else{
					drs_steering_state = DRS_STEERING_RIGHT_TRIPPED;
				}
				return true; //DRS should be closed
			}
			return false;	//DRS should be open
	}
	//reaches here we are in trouble
	return false;	//DRS should be open
}