// Set up header file for Efuse with function prorotypes and variables as needed

#ifndef INC_Efuse_H_
#define INC_Efuse_H_

#include "main.h"
#include "GopherCAN.h"


#define NUM_OF_CHANNELS 3

//states of efuse
#define ENABLED 0
#define TRIPPED 1
#define SHUTDOWN 2

//States of current types for current calculation
#define TPS2552 0
#define TPS259630 1


typedef struct {
    FLOAT_CAN_STRUCT* parameter;
    GPIO_TypeDef* enable_fuse_port;
    uint16_t enable_fuse_pin;
    GPIO_TypeDef* flt_out_port;
    uint16_t flt_out_pin;
    GPIO_TypeDef* flt_led_port;
    uint16_t flt_led_pin;
    ADC_TypeDef* curr_out_port;
    uint16_t curr_out_pin;
    uint8_t enabled;
    uint8_t adc_type;
    float amp_max;
    uint8_t flt_state;
    uint32_t trip_start;
    uint32_t reset_delay_ms;
    uint8_t state;
    uint32_t last_update;
    uint8_t overcurrent_count;
    uint8_t max_overcurrent_count;                      //amount of retries before disabling
    U8_CAN_STRUCT* overcurrentparam;
    U8_CAN_STRUCT* overcurrentcountparam;
} RVC_POWER_CHANNEL;


extern RVC_POWER_CHANNEL* POWER_CHANNELS[NUM_OF_CHANNELS];

void update_efuse(RVC_POWER_CHANNEL* efuse);

#endif /* INC_Efuse_H_ */