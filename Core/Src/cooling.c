// Write 2 functions:
// one that turns on/off the pumps & fans with some hysteresis
// one that turns uses PWM togive variable fan & pump speed with P control (might need a helper function for this)
//Cooling variables
#include "cooling.h"

//Fan
U8 rad_fan_F_state = RAD_FAN_OFF;
U8 rad_fan_R_state = RAD_FAN_OFF;

TIM_HandleTypeDef* FAN_PWM_Timer_F;
TIM_HandleTypeDef* FAN_PWM_Timer_R;
U32 FAN_Channel_F;
U32 FAN_Channel_R;
float fan_percent;

//Pump
// boolean steady_temperatures_achieved_pump[] = {true, true}; //LOT if pump temperatures have returned to ready state
// U8 pump_readings_below_HYS_threshold = 0;

U8 digital_pump_F_state = PUMP_DIGITAL_OFF; //if no pump pwm and just digital
U8 digital_pump_R_state = PUMP_DIGITAL_OFF; //if no pump pwm and just digital

TIM_HandleTypeDef* PUMP_PWM_Timer_F;
TIM_HandleTypeDef* PUMP_PWM_Timer_R;
U32 PUMP_Channel_F;
U32 PUMP_Channel_R;
float pump_percent;

void init_Pumps(TIM_HandleTypeDef* timer_address_F, TIM_HandleTypeDef* timer_address_R, U32 channel_F, U32 channel_R){
	PUMP_PWM_Timer_F = timer_address_F;
	PUMP_PWM_Timer_R= timer_address_R;
	PUMP_Channel_F = channel_F;
	PUMP_Channel_R = channel_R;
	HAL_TIM_PWM_Start(PUMP_PWM_Timer_F, PUMP_Channel_F); //turn on PWM generation
	HAL_TIM_PWM_Start(PUMP_PWM_Timer_R, PUMP_Channel_R); //turn on PWM generation
}

void init_Fans(TIM_HandleTypeDef* timer_address_F, TIM_HandleTypeDef* timer_address_R, U32 channel_F, U32 channel_R){
	FAN_PWM_Timer_F = timer_address_F;
	FAN_PWM_Timer_R = timer_address_R;
	FAN_Channel_F = channel_F;
	FAN_Channel_R = channel_R;
	HAL_TIM_PWM_Start(FAN_PWM_Timer_F, FAN_Channel_F); //turn on PWM generation
	HAL_TIM_PWM_Start(FAN_PWM_Timer_R, FAN_Channel_R); //turn on PWM generation
}

void update_fans(bool next_state){
	if(next_state){
		__HAL_TIM_SET_COMPARE(FAN_PWM_Timer_R, FAN_Channel_R, 95000);
		__HAL_TIM_SET_COMPARE(FAN_PWM_Timer_F, FAN_Channel_F, 95000);
		//75 % DUT
	}
	else{
		__HAL_TIM_SET_COMPARE(FAN_PWM_Timer_R, FAN_Channel_R, 50000);
		__HAL_TIM_SET_COMPARE(FAN_PWM_Timer_F, FAN_Channel_F, 50000);
		//50 % DUT
	}
	__HAL_TIM_SET_COMPARE(PUMP_PWM_Timer_F, PUMP_Channel_F, 25000);
	__HAL_TIM_SET_COMPARE(PUMP_PWM_Timer_R, PUMP_Channel_R, 25000);
}

void update_cooling_on_off() {
	//motor_mph = electricalRPM_erpm.data * DRIVE_RATIO;
	//max temps used for now, can change to by pump and fan if needed
	float inv_temp_front = fmaxf(controllerTemp_RL_C.data, controllerTemp_RR_C.data);
	float inv_temp_rear = fmaxf(controllerTemp_FL_C.data, controllerTemp_FR_C.data);
	float motor_temp_front = fmaxf(motorTemp_RL_C.data, motorTemp_RR_C.data);
	float motor_temp_rear = fmaxf(motorTemp_FL_C.data, motorTemp_FR_C.data);

	//pump states front
	if ((inv_temp_front > INVERTER_PUMP_POWER_ON_THRESH) || (motor_temp_front > MOTOR_PUMP_THRESH_C)) {
			digital_pump_F_state = PUMP_DIGITAL_ON;
	} else if ((inv_temp_front < INVERTER_PUMP_POWER_ON_THRESH - COOLING_HYSTERESIS_C) && (motor_temp_front < MOTOR_PUMP_THRESH_C - COOLING_HYSTERESIS_C)) {
			digital_pump_F_state = PUMP_DIGITAL_OFF;
	}

	//pump states rear
	if((inv_temp_rear > INVERTER_PUMP_POWER_ON_THRESH) || (motor_temp_rear > MOTOR_PUMP_THRESH_C)) {
			digital_pump_R_state = PUMP_DIGITAL_ON;
	} else if ((inv_temp_rear < INVERTER_PUMP_POWER_ON_THRESH - COOLING_HYSTERESIS_C) && (motor_temp_rear < MOTOR_PUMP_THRESH_C - COOLING_HYSTERESIS_C)) {
			digital_pump_R_state = PUMP_DIGITAL_OFF;
	}

	//radiator fan front
	if ((inv_temp_front > INVERTER_FAN_THRESH_C) || (motor_temp_front > MOTOR_FAN_THRESH_C)) {
			rad_fan_F_state = RAD_FAN_ON;
	} else if ((inv_temp_front < INVERTER_FAN_THRESH_C - COOLING_HYSTERESIS_C) && (motor_temp_front < MOTOR_FAN_THRESH_C - COOLING_HYSTERESIS_C)) {
			rad_fan_F_state = RAD_FAN_OFF;
	}

	//radiator fan rear
	if ((inv_temp_rear > INVERTER_FAN_THRESH_C) || (motor_temp_rear > MOTOR_FAN_THRESH_C)) {
			rad_fan_R_state = RAD_FAN_ON;
	} else if ((inv_temp_rear < INVERTER_FAN_THRESH_C - COOLING_HYSTERESIS_C) && (motor_temp_rear < MOTOR_FAN_THRESH_C - COOLING_HYSTERESIS_C)) {
			rad_fan_R_state = RAD_FAN_OFF;
	}

	__HAL_TIM_SET_COMPARE(PUMP_PWM_Timer_F, PUMP_Channel_F, PUMP_COUNTER_PERIOD * digital_pump_F_state);
	__HAL_TIM_SET_COMPARE(PUMP_PWM_Timer_R, PUMP_Channel_R, PUMP_COUNTER_PERIOD * digital_pump_R_state);
	__HAL_TIM_SET_COMPARE(FAN_PWM_Timer_F, FAN_Channel_F, FAN_COUNTER_PERIOD * rad_fan_F_state);
	__HAL_TIM_SET_COMPARE(FAN_PWM_Timer_R, FAN_Channel_R, FAN_COUNTER_PERIOD * rad_fan_R_state);
}

void update_cooling_dynamic() {
	//simple hysteresis control for fans and pumps
	//motor_mph = electricalRPM_erpm.data * DRIVE_RATIO;
	//max temps used for now, can change to by pump and fan if needed
	float inv_temp_front = fmaxf(controllerTemp_RL_C.data, controllerTemp_RR_C.data);
	float inv_temp_rear = fmaxf(controllerTemp_FL_C.data, controllerTemp_FR_C.data);
	float motor_temp_front = fmaxf(motorTemp_RL_C.data, motorTemp_RR_C.data);
	float motor_temp_rear = fmaxf(motorTemp_FL_C.data, motorTemp_FR_C.data);

	float above_temp_front = fmaxf(inv_temp_front - INVERTER_FAN_THRESH_C, motor_temp_front - MOTOR_FAN_THRESH_C);
	float above_temp_rear = fmaxf(inv_temp_rear - INVERTER_FAN_THRESH_C, motor_temp_rear - MOTOR_FAN_THRESH_C);

	//pump states front
	if (above_temp_front > 0) {
			digital_pump_F_state = PUMP_DIGITAL_ON;
	} else if (above_temp_front + COOLING_HYSTERESIS_C < 0) {
			digital_pump_F_state = PUMP_DIGITAL_OFF;
	}

	//pump states rear
	if (above_temp_rear > 0) {
			digital_pump_R_state = PUMP_DIGITAL_ON;
	} else if (above_temp_rear + COOLING_HYSTERESIS_C < 0) {
			digital_pump_R_state = PUMP_DIGITAL_OFF;
	}

	//radiator fan front
	if (above_temp_front > 0) {
			rad_fan_F_state = FAN_PERCENT_LINEAR * above_temp_front;
	} else if (above_temp_front + COOLING_HYSTERESIS_C < 0) {
			rad_fan_F_state = RAD_FAN_OFF;
	}

	//radiator fan rear
	if (above_temp_rear > 0) {
			rad_fan_R_state = FAN_PERCENT_LINEAR * above_temp_rear;
	} else if (above_temp_rear + COOLING_HYSTERESIS_C < 0) {
			rad_fan_R_state = RAD_FAN_OFF;
	}

	if(rad_fan_F_state > FAN_MAX_PERCENT){
		rad_fan_F_state = FAN_MAX_PERCENT;
	}
	if(rad_fan_R_state > FAN_MAX_PERCENT){
		rad_fan_R_state = FAN_MAX_PERCENT;
	}

	__HAL_TIM_SET_COMPARE(PUMP_PWM_Timer_F, PUMP_Channel_F, PUMP_COUNTER_PERIOD * digital_pump_F_state / 100);
	__HAL_TIM_SET_COMPARE(PUMP_PWM_Timer_R, PUMP_Channel_R, PUMP_COUNTER_PERIOD * digital_pump_R_state / 100);
	__HAL_TIM_SET_COMPARE(FAN_PWM_Timer_F, FAN_Channel_F, FAN_COUNTER_PERIOD * rad_fan_F_state / 100);
	__HAL_TIM_SET_COMPARE(FAN_PWM_Timer_R, FAN_Channel_R, FAN_COUNTER_PERIOD * rad_fan_R_state / 100);
}