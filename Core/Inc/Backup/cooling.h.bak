#include "rvc.h"

// ====================================== PULLUP PARAMETERS =====================================
#define RAD_FAN_ON (GPIO_PIN_SET)
#define RAD_FAN_OFF (GPIO_PIN_RESET)
#define PUMP_DIGITAL_ON (GPIO_PIN_SET)
#define PUMP_DIGITAL_OFF (GPIO_PIN_RESET)
// ==============================================================================================


// ====================================== COOLING PARAMETERS ====================================
#define INVERTER_PUMP_POWER_ON_THRESH 45.0f //Low Threshold Turn On
#define MOTOR_PUMP_THRESH_C         45.0f // Motor temperature at which the cooling system turns on

//#define temperature thresholds
#define INVERTER_FAN_THRESH_C    50.0f  // Inverter temperature at which the cooling system turns on
#define MOTOR_FAN_THRESH_C       70.0f  // Motor temperature at which the cooling system turns on
#define MOTOR_MAX_THRESH_C       120.0f // Motor temperature at which the cooling system is at max power
#define COOLING_HYSTERESIS_C     5.0f   // Hysteresis when confined to digital signal (on/off)
#define CAR_SPEED_FAN_HYS	     5.0f   // Hysteresis on Car speed for turning on/off fans
#define CAR_SPEED_FAN_THRESH	 20.0f  // Car speed at which air cooling from movement is enough
//#define Fan
#define FAN_OFF                 0       //0% duty cycle --> 0/49999
#define FAN_MIN_PERCENT         25      //Minimum fan speed (25% DUT ~2500rpm)
#define FAN_MAX_PERCENT         100     //Maximum fan speed (100% DUT ~6400rpm)
#define FAN_PERCENT_LINEAR      5       //Percent per degree C for linear fan speed 
#define FAN_PWM_FREQ            25000   //recommended PWM frequency
//#define USING_PUMP_PWM
#define PUMP_OFF            0     //0% duty cycle --> 0/49999
#define PUMP_50_PERCENT     25000 //50% duty cycle --> 25000/49999
#define PUMP_100_PERCENT    49999 //100% duty cycle --> 49999/49999
#define PUMP_COUNTER_PERIOD	49999
#define PUMP_PERCENT_OFFSET 0.5f //if the pump is on it will statr at 50%
//#define PI control parameters
#define COOLING_OFF_DELAY_MS 5000  //delay after temps go below threshold to turn off cooling system
#define K_Integral          0.1    //
#define K_Porportional      5      //
// ==============================================================================================



void update_cooling_simple();   // Hysteresis cooling code (go4-25)
void update_cooling_dynamic();  // Control loop for cooling
void update_pwm(TIM_HandleTypeDef* timer_address, U32 channel, float percent);
void init_Pumps(TIM_HandleTypeDef* timer_address_0, TIM_HandleTypeDef* timer_address_1, U32 channel_0, U32 channel_1);
void init_Fans(TIM_HandleTypeDef* timer_address_0, TIM_HandleTypeDef* timer_address_1, U32 channel_0, U32 channel_1);