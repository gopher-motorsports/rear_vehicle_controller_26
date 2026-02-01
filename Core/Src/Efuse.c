// TODO: Write an Efuse Function with two states
#include "EFuse.h"

uint8_t SNS_5V_1_FLT;
uint8_t SNS_5V_2_FLT;
uint8_t SNS_12V_FLT;

//Basic 5V Efuse setup
EFUSE_DATA* SNS_5V_1 = {
    .enable_fuse_port = SNS_5V_1_EN_GPIO_PORT;
    .enable_fuse_pin = SNS_5V_1_EN_PIN;
    .flt_out_port = SNS_5V_1_FLT_GPIO_PORT;
    .flt_out_pin = SNS_5V_1_FLT_PIN;
    .flt_led_port = FLT_5V_1_LED_GPIO_PORT;
    .flt_led_pin = FLT_5V_1_LED_PIN;
    .flt = 0;
    .reset_delay_ms = 1000;
    .state = ENABLED;
    .last_update = 0;
    .flt_count = 0;
    .max_flt_count = 3;
} ;

EFUSE_DATA* SNS_5V_1 = {
    .enable_fuse_port = SNS_5V_2_EN_GPIO_PORT;
    .enable_fuse_pin = SNS_5V_2_EN_PIN;
    .flt_out_port = SNS_5V_2_FLT_GPIO_PORT;
    .flt_out_pin = SNS_5V_2_FLT_PIN;
    .flt_led_port = FLT_5V_2_LED_GPIO_PORT;
    .flt_led_pin = FLT_5V_2_LED_PIN;
    .flt = 0;
    .reset_delay_ms = 1000;
    .state = ENABLED;
    .last_update = 0;
    .flt_count = 0;
    .max_flt_count = 3;
} ;

EFUSE_DATA* SNS_12V = {
    .enable_fuse_port = SNS_12V_EN_GPIO_PORT;
    .enable_fuse_pin = SNS_12V_EN_PIN;
    .flt_out_port = SNS_12V_FLT_GPIO_PORT;
    .flt_out_pin = SNS_12V_FLT_PIN;
    .flt_led_port = FLT_12V_LED_GPIO_PORT;
    .flt_led_pin = FLT_12V_LED_PIN;
    .curr_out_port = SNS_12V_ILM_GPIO_Port;
    .curr_out_pin = SNS_12V_ILM_PIN;
    .curr_limit = 0.6;
    .flt = 0;
    .reset_delay_ms = 1000;
    .state = ENABLED;
    .last_update = 0;
    .flt_count = 0;
    .max_flt_count = 3;
} ;


void update_efuse(EFUSE_DATA* efuse) {
    efuse.flt = HAL_GPIO_ReadPin(efuse.flt_out_port, efuse.flt_pin);
    uint32_t tick = HAL_GetTick();

    //flips to tripped if enabled and flt asserted
    if(efuse.flt && (efuse.state == ENABLED)){
        efuse.fltcount ++;
        efuse.state = TRIPPED;
        HAL_GPIO_WritePin(efuse.enable_fuse_port, efuse.enable_fuse_pin, 0);
        HAL_GPIO_WritePin(efuse.flt_led_port, efuse.flt_led_pin, 0);
        efuse.last_update = tick;
    } 

    //ilm current is checked for 12V, flt only asserts for ilm-gnd short and overtemp
    if((efuse.curr_limit != 0) && (efuse.state == ENABLED)){
        if(12VElectronicFuseCurrent_mamps.data > 450){
            efuse.fltcount ++;
            efuse.state = TRIPPED;
            HAL_GPIO_WritePin(efuse.enable_fuse_port, efuse.enable_fuse_pin, 0);
            HAL_GPIO_WritePin(efuse.flt_led_port, efuse.flt_led_pin, 0);
            efuse.last_update = tick;
        } else{
            efuse.last_update = tick;
        }
    }

    //if in tripped state and delay has finished
    if((efuse.state == TRIPPED) && (tick - efuse.last_update > efuse.reset_delay_ms)){
        if(efuse.flt){
            efuse.fltcount ++;
        } else{ //flips back leds
            efuse.state = ENABLED;
            HAL_GPIO_WritePin(efuse.enable_fuse_port, efuse.enable_fuse_pin, 1);
            HAL_GPIO_WritePin(efuse.flt_led_port, efuse.flt_led_pin, 1);
        }
        efuse.last_update = tick;
    }

    //shutdown catch
    if(!(efuse.state == SHUTDOWN) && (efuse.flt_cout >= efuse.max_flt_count)){
        efuse.state == SHUTDOWN;
        efuse.last_update = tick;
    }
}