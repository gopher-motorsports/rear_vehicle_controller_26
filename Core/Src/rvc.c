#include "rvc.h"

// the HAL_CAN struct. This example only works for a single CAN bus
CAN_HandleTypeDef* example_hcan;
#define PRINTF_HB_MS_BETWEEN 500

// the CAN callback function used in this example
static void change_led_state(U8 sender, U8 remote_param, U8 UNUSED1, U8 UNUSED2, U8 UNUSED3);
static void init_error(void);

// init
//  What needs to happen on startup in order to run GopherCAN
void init(CAN_HandleTypeDef* hcan_ptr)
{
	example_hcan = hcan_ptr;

	// initialize CAN
	if (init_can(hcan_ptr, GCAN0))
	{
		
	}

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
	U8 button_state;

	// send the current tick over UART every second
	if (HAL_GetTick() - last_print_hb >= PRINTF_HB_MS_BETWEEN)
	{
		HAL_GPIO_TogglePin(GRN_LED_GPIO_Port, GRN_LED_Pin);
		//printf("Current tick: %lu\n", HAL_GetTick());
		last_print_hb = HAL_GetTick();
	}
}

// can_callback_function example
static void change_led_state(U8 sender, U8 remote_param, U8 UNUSED1, U8 UNUSED2, U8 UNUSED3)
{
	HAL_GPIO_WritePin(GRN_LED_GPIO_Port, GRN_LED_Pin, !!remote_param);
	return;
}

// init_error
//  This function will stay in an infinite loop, blinking the LED in a 0.5sec period. Should only
//  be called from the init function before the RTOS starts
void init_error(void)
{
	while (1)
	{
		HAL_GPIO_TogglePin(GRN_LED_GPIO_Port, GRN_LED_Pin);
		HAL_Delay(250);
	}
}

// Port over Buzzer & BrakeLight Code from RVC 2025
// Buzzer should be active when inverters are in predrive, brake light should be active about a PSI threshold (~25psi)
void update_brakelight_and_buzzer(){
	if(brakePressureRear_psi.data > BRAKE_LIGHT_THRESH_psi) {
		HAL_GPIO_WritePin(BRK_LT_GPIO_Port, BRK_LT_Pin, MOSFET_PULL_DOWN_ON);
		update_and_queue_param_u8(&brakeLightOn_state, TRUE);
	} else {
		HAL_GPIO_WritePin(BRK_LT_GPIO_Port, BRK_LT_Pin, MOSFET_PULL_DOWN_OFF);
		update_and_queue_param_u8(&brakeLightOn_state, FALSE);
	}

	if(vehicleState_state.data == VEHICLE_PREDRIVE) {
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, MOSFET_PULL_DOWN_ON);
		HAL_GPIO_WritePin(PCB_BUZZER_GPIO_Port, PCB_BUZZER_Pin, PCB_BUZZ_ON);
		update_and_queue_param_u8(&vehicleBuzzerOn_state, TRUE);
	} else {
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, MOSFET_PULL_DOWN_OFF);
		HAL_GPIO_WritePin(PCB_BUZZER_GPIO_Port, PCB_BUZZER_Pin, PCB_BUZZ_OFF);
		update_and_queue_param_u8(&vehicleBuzzerOn_state, FALSE);
	}
	return;
}