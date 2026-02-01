// Set up header file for Efuse with function prorotypes and variables as needed

#ifndef INC_Efuse_H_
#define INC_Efuse_H_

#include "main.h"
#include "GopherCAN.h"

//states of efuse
#define ENABLED 0
#define TRIPPED 1
#define SHUTDOWN 2


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
    float curr_limit = 0.0;
    uint8_t flt;
    uint32_t reset_delay_ms;
    uint8_t state;
    uint32_t last_update;
    uint8_t flt_count;
    uint8_t max_flt_count;                      //amount of retries before disabling
    U8_CAN_STRUCT* overcurrentparam;
    U8_CAN_STRUCT* overcurrentcountparam;
} EFUSE_DATA;


#endif /* INC_Efuse_H_ */