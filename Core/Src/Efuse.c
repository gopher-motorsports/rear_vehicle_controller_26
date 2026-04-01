// TODO: Write an Efuse Function with two states
#include "EFuse.h"

uint8_t SNS_5V_1_FLT;
uint8_t SNS_5V_2_FLT;
uint8_t SNS_12V_FLT;

//Basic 5V Efuse setup
RVC_POWER_CHANNEL ch_5v_1 = {
    .enable_fuse_port = SNS_5V_1_EN_GPIO_Port,
    .enable_fuse_pin = SNS_5V_1_EN_Pin,
    .flt_out_port = SNS_5V_1_FLT_GPIO_Port,
    .flt_out_pin = SNS_5V_1_FLT_Pin,
    .flt_led_port = FLT_5V_1_LED_GPIO_Port,
    .flt_led_pin = FLT_5V_1_LED_Pin,
    .amp_max = 0.250f, //Done by hardware at 250mA
    .enabled = ENABLED,
    .adc_type = TPS2552,
    .trip_start = 0,
    .reset_delay_ms = 1000,
    .last_update = 0,
    .overcurrent_count = 0,
    .max_overcurrent_count = 3,
    .overcurrentparam = &rvc5V1Overcurrent,
	.overcurrentcountparam = &rvc5V1Overcurrent_Count
} ;

RVC_POWER_CHANNEL ch_5v_2 = {
    .enable_fuse_port = SNS_5V_2_EN_GPIO_Port,
    .enable_fuse_pin = SNS_5V_2_EN_Pin,
    .flt_out_port = SNS_5V_2_FLT_GPIO_Port,
    .flt_out_pin = SNS_5V_2_FLT_Pin,
    .flt_led_port = FLT_5V_2_LED_GPIO_Port,
    .flt_led_pin = FLT_5V_2_LED_Pin,
    .amp_max = 0.250f, //Done by hardware at 250mA
    .enabled = ENABLED,
    .adc_type = TPS2552,
    .trip_start = 0,
    .reset_delay_ms = 1000,
    .last_update = 0,
    .overcurrent_count = 0,
    .max_overcurrent_count = 3,
    .overcurrentparam = &rvc5V2Overcurrent,
    .overcurrentcountparam = &rvc5V2Overcurrent_Count
} ;

RVC_POWER_CHANNEL ch_12v_0 = {
    .currentparam = &rvc12VCurrent_A,
    .enable_fuse_port = SNS_12V_EN_GPIO_Port,
    .enable_fuse_pin = SNS_12V_EN_Pin,
    .flt_out_port = SNS_12V_FLT_GPIO_Port,
    .flt_out_pin = SNS_12V_FLT_Pin,
    .flt_led_port = FLT_12V_LED_GPIO_Port,
    .flt_led_pin = FLT_12V_LED_Pin,
    .amp_max = 0.6,
    .enabled = ENABLED,
    .adc_type = TPS259630,
    .trip_start = 0,
    .reset_delay_ms = 1000,
    .last_update = 0,
    .overcurrent_count = 0,
    .max_overcurrent_count = 3,
    .overcurrentparam = &rvc12VOvercurrent,
    .overcurrentcountparam = &rvc12VOvercurrent_Count
} ;

RVC_POWER_CHANNEL* POWER_CHANNELS[NUM_OF_CHANNELS] = {
    &ch_12v_0,
    &ch_5v_1,
    &ch_5v_2
};


void update_efuse(RVC_POWER_CHANNEL* efuse) {
    uint32_t tick = HAL_GetTick();
    efuse->last_update = tick;

    efuse->flt_state = HAL_GPIO_ReadPin(efuse->flt_out_port, efuse->flt_out_pin);
    uint8_t next_state = efuse->state;    

    //handle state change logic
    switch(next_state){
        case ENABLED:
            //check if we need to trip the fuse
            if(efuse->flt_state == 1){
                        next_state = TRIPPED;
            }
            else{
                if(efuse->currentparam->data > efuse->amp_max){
                    next_state = TRIPPED;
                }
            }

            if(next_state == TRIPPED){
                efuse->overcurrent_count ++;
                efuse->trip_start = tick;
                if(efuse->overcurrent_count >= efuse->max_overcurrent_count){
                    next_state = SHUTDOWN;
                }
            }
            break;
        case TRIPPED:
            if(tick - efuse->last_update > efuse->reset_delay_ms){
                next_state = ENABLED;
            }
            break;
        case SHUTDOWN:
            break;
    }
    
    //handle state change
    switch(next_state){
        case ENABLED:
            efuse->state = ENABLED;
            HAL_GPIO_WritePin(efuse->enable_fuse_port, efuse->enable_fuse_pin, 1);
            HAL_GPIO_WritePin(efuse->flt_led_port, efuse->flt_led_pin, 0);
            break;
        case TRIPPED:
            efuse->state = TRIPPED;
            HAL_GPIO_WritePin(efuse->enable_fuse_port, efuse->enable_fuse_pin, 0);
            HAL_GPIO_WritePin(efuse->flt_led_port, efuse->flt_led_pin, 1);
            break;      
        case SHUTDOWN:
            efuse->state = SHUTDOWN;
            HAL_GPIO_WritePin(efuse->enable_fuse_port, efuse->enable_fuse_pin, 0);
            HAL_GPIO_WritePin(efuse->flt_led_port, efuse->flt_led_pin, 1);
            break;  
    }
}