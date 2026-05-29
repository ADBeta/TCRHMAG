/******************************************************************************
* System Initialisation and Hardware Handling      (TCRHMAG)
*
* Ver 0.1
* (c) ADBeta
******************************************************************************/
#ifndef TCRHMAG_SYSINIT_H
#define TCRHMAG_SYSINIT_H


#include <ch32fun.h>

/*** Definitions *************************************************************/
#define IWDG_KEY_CHANGE    0x5555
#define IWDG_KEY_ENABLE    0xCCCC
#define IWDG_KEY_FEED      0xAAAA


// External 1ms SysTick counter variable
extern volatile uint32_t _systick_millis;




/*** Watchdog Functions ******************************************************/
/// @brief Initialise the IWDG
/// @param prescaler, the IWDG Clock Prescaler Value
/// @param rst_val, the IWDG Reset Value
/// @return None
void iwdg_init(const uint8_t prescaler, const uint16_t rst_val);


/// @brief Feeds the watchdog and keeps him happy :)
/// @param None
/// @return None
__attribute__((always_inline))
static inline void iwdg_feed(void)
{ IWDG->CTLR = IWDG_KEY_FEED; }



/*** Systick Functions *******************************************************/
/// @brief Initialises the 1ms Systick
/// @param None
/// @return None
void systick_init(void);


/// @brief The IRQ for the 1ms Systick
/// @param None
/// @return None
__attribute__((interrupt))
void SysTick_Handler(void);



/*** PWM Functions ***********************************************************/
/// @brief Initialises PWM for TIM2 Channel 2 (PD3) and TIM2 Channel 3 (PC0)
/// @param None
/// @return None
void pwm_init(void);


#endif
