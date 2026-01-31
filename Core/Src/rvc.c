#include "rvc.h"
#include "cooling.h"


//Current Sensing Variables:
uint8_t currentSensorStatus = UNINITIALIZED;

//BSPD Variables:
uint8_t rvcBspdRunawayFault = 0;
uint8_t rvcBspdInputFault = 0;
uint8_t rvcBspdFaultActive = 0;
uint8_t rvcBspdFaultLatched = 0;

// the HAL_CAN struct. This example only works for a single CAN bus
CAN_HandleTypeDef* example_hcan;
#define PRINTF_HB_MS_BETWEEN 500

// the CAN callback function used in this example
static void change_led_state(U8 sender, U8 remote_param, U8 UNUSED1, U8 UNUSED2, U8 UNUSED3);
static void init_error(void);

// init
//  What needs to happen on startup in order to run GopherCAN
void init_rvc(CAN_HandleTypeDef* hcan_ptr)
{
	example_hcan = hcan_ptr;

	// initialize CAN
	if (init_can(hcan_ptr, GCAN0))
	{
		
	}
	init_Fans(&htim2, &htim2, TIM_CHANNEL_1, TIM_CHANNEL_2);
	init_Pumps(&htim3, &htim3, TIM_CHANNEL_1, TIM_CHANNEL_2);

	// Set the function pointer of SET_LED_STATE. This means the function change_led_state()
	// will be run whenever this can command is sent to the module
	attach_callback_cmd(SET_LED_STATE, &change_led_state);
}


// can_buffer_handling_loop
//  This loop will handle CAN RX software task and CAN TX hardware task. Should be
//  called every 1ms or as often as received messages should be handled
void can_buffer_handling_loop()
{
	// handle each RX message in the buffer
	if (service_can_rx_buffer())
	{
		// an error has occurred
	}

	// handle the transmission hardware for each CAN bus
	service_can_tx(example_hcan);
}


// main_loop
//  another loop. This includes logic for sending a CAN command. Designed to be
//  called every 10ms
void main_loop()
{
	static U32 last_print_hb = 0;

	// send the current tick over UART every second
	if (HAL_GetTick() - last_print_hb >= PRINTF_HB_MS_BETWEEN)
	{
		HAL_GPIO_TogglePin(HBEAT_LED_GPIO_Port, HBEAT_LED_Pin);
		//printf("Current tick: %lu\n", HAL_GetTick());
		last_print_hb = HAL_GetTick();
	}
}

// can_callback_function example
static void change_led_state(U8 sender, U8 remote_param, U8 UNUSED1, U8 UNUSED2, U8 UNUSED3)
{
	HAL_GPIO_WritePin(HBEAT_LED_GPIO_Port, HBEAT_LED_Pin, !!remote_param);
	return;
}

// init_error
//  This function will stay in an infinite loop, blinking the LED in a 0.5sec period. Should only
//  be called from the init function before the RTOS starts
void init_error(void)
{
	while (1)
	{
		HAL_GPIO_TogglePin(HBEAT_LED_GPIO_Port, HBEAT_LED_Pin);
		HAL_Delay(250);
	}
}

// #TODO: write a function that sends highly important signals @ 250 Hz, medium important signals @ 50 Hz, and low importance signals @ 10 Hz
// This one ain't to deep just call update_and_queue_u8(), update_and_queue_u16(), update_and_queue_float() on everything
// You can actually just run this function in the main_loop() called at 1ms
// Make sure you are passing in global vairables to this file not the .data of the variable, that will send the parameter super slow (2.5s)
// For ADC parameters you can ignore having to call "update_and_queue()" specifically as Gsense does it in the backend
void update_telemtry_params(){
	// BSPD Faults

	// Efuse Sensor Power Faults (Not the fault pins, but if sensor power is disabled)

	// Cooling Power (Pump, Radiator Fan)

	// Status(Hbeat, Brake Light, Buzzer)
}


// #TODO: write a function that sends highly important signals @ 250 Hz, medium important signals @ 50 Hz, and low importance signals @ 10 Hz
// This one ain't to deep just call update_and_queue_u8(), update_and_queue_u16(), update_and_queue_float() on everything
// You can actually just run this function in the main_loop() called at 1ms
// Make sure you are passing in global vairables to this file not the .data of the variable, that will send the parameter super slow (2.5s)
// For ADC parameters you can ignore having to call "update_and_queue()" specifically as Gsense does it in the backend
void update_telemtry_params(){
	// BSPD Faults

	// Efuse Sensor Power Faults (Not the fault pins, but if sensor power is disabled)

	// Cooling Power (Pump, Radiator Fan)

	// Status(Hbeat, Brake Light, Buzzer)
}

// Port over Buzzer & BrakeLight Code from RVC 2025
// Buzzer should be active when inverters are in predrive, brake light should be active about a PSI threshold (~25psi)
void update_brakelight_and_buzzer(){
	if(rvcBrakePressureRear_psi.data > BRAKE_LIGHT_THRESH_psi) {
		HAL_GPIO_WritePin(BRK_LIGHT_GPIO_Port, BRK_LIGHT_Pin, MOSFET_PULL_DOWN_ON);
		update_and_queue_param_u8(&rvcBrakeLightOn_state, TRUE);
	} else {
		HAL_GPIO_WritePin(BRK_LIGHT_GPIO_Port, BRK_LIGHT_Pin, MOSFET_PULL_DOWN_OFF);
		update_and_queue_param_u8(&rvcBrakeLightOn_state, FALSE);
	}

	if(fvcVehicleState_state.data == VEHICLE_PREDRIVE) {
		HAL_GPIO_WritePin(RTD_BUZZER_GPIO_Port, RTD_BUZZER_Pin, MOSFET_PULL_DOWN_ON);
		update_and_queue_param_u8(&rvcVehicleBuzzerOn_state, TRUE);
	} else {
		HAL_GPIO_WritePin(RTD_BUZZER_GPIO_Port, RTD_BUZZER_Pin, MOSFET_PULL_DOWN_OFF);
		update_and_queue_param_u8(&rvcVehicleBuzzerOn_state, FALSE);
	}
	return;
}

void update_TSSI_LED(){
	if(HAL_GetTick() > TSSI_RESET_TIME_ms && (imdFault_state.data || amsFault_state.data)) {
		HAL_GPIO_WritePin(TSSI_GREEN_GPIO_Port, TSSI_GREEN_Pin, 0);
		HAL_GPIO_WritePin(TSSI_RED_GPIO_Port, TSSI_RED_Pin, (HAL_GetTick() % TSSI_FLASH_PERIOD_ms) < TSSI_FLASH_PERIOD_ms / 2);
	}
	else{
		HAL_GPIO_WritePin(TSSI_RED_GPIO_Port, TSSI_RED_Pin, 0);
		HAL_GPIO_WritePin(TSSI_GREEN_GPIO_Port, TSSI_GREEN_Pin, 1);
	}
}

float getTractiveSystemCurrent(){
    // Fetch current sensor data from gophercan
	float tractiveSystemCurrent = 0;
    float currHI = rvcCurrentSensorHigh_A.data;
    float currLO = rvcCurrentSensorLow_A.data;
    uint8_t currentSensorStatusHI = 0;
    uint8_t currentSensorStatusLO = 0;

    // If the current exceeds the following threshold in either the positive or negative direction,
    // the sensor input has railed to 0 or 5v and a current sensor error is set
    currentSensorStatusHI = (fabs(currHI) < CURRENT_HIGH_RAIL_THRESHOLD) ? (WORKING) : (FAULTING);
    currentSensorStatusLO = (fabs(currLO) < CURRENT_LOW_RAIL_THRESHOLD) ? (WORKING) : (FAULTING);

    // To use the HI current sensor channel, it must be working AND (it must exceed the measuring range of the low channel OR the low channel must be faulty)
    if ((currentSensorStatusHI == WORKING) && ((fabs(currLO) > CURRENT_LOW_TO_HIGH_SWITCH_THRESHOLD + (CHANNEL_FILTERING_WIDTH / 2)) || (currentSensorStatusLO != WORKING)))
    {
        tractiveSystemCurrent = currHI;
        currentSensorStatus = WORKING;
    }
    else if ((currentSensorStatusHI == WORKING) && (currentSensorStatusLO == WORKING) && ((fabs(currLO) > CURRENT_LOW_TO_HIGH_SWITCH_THRESHOLD - (CHANNEL_FILTERING_WIDTH / 2))))
    {
        float interpolationStart    =   CURRENT_LOW_TO_HIGH_SWITCH_THRESHOLD  - (CHANNEL_FILTERING_WIDTH / 2);
        float interpolationRatio    =   (currLO - interpolationStart) / CHANNEL_FILTERING_WIDTH;
        float filteredCurrent       =   ((1.0f - interpolationRatio) * currLO) + (interpolationRatio * currHI);
        tractiveSystemCurrent  =   filteredCurrent;
        currentSensorStatus = WORKING;
    }
    else if (currentSensorStatusLO == WORKING) // If the above condition is not satisfied, the LO channel must be working in order to use its data
    {
        tractiveSystemCurrent = currLO;
        currentSensorStatus = WORKING;
    }
    else // If both sensors are faulty, no current data can be accurately returned
    {
    	currentSensorStatus = FAULTING;
    }

    return tractiveSystemCurrent;
}



// Create function to update BSPD parmameters
void update_bspd_params() {
	rvcBspdRunawayFault = HAL_GPIO_ReadPin(TS_ACC_FLT_GPIO_Port, TS_ACC_FLT_Pin);
	rvcBspdFaultActive = HAL_GPIO_ReadPin(TS_SNS_FLT_GPIO_Port, TS_SNS_FLT_Pin);
	rvcBspdFaultLatched = HAL_GPIO_ReadPin(BSPD_Logic_Output_GPIO_Port, BSPD_Logic_Output_Pin);
	// If the fault is active, but not runaway, then it must be an input fault
	rvcBspdInputFault = !rvcBspdRunawayFault && rvcBspdFaultActive;
}