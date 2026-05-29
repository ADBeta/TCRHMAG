/******************************************************************************
* System Initialisation and Hardware Handling      (TCRHMAG)
*
* (c) ADBeta
******************************************************************************/
#include "sysinit.h"
#include <ch32fun.h>



/*** Watchdog Functions ******************************************************/
void iwdg_init(const uint8_t prescaler, const uint16_t rst_val)
{
	// Enable changes then set prescaler
	IWDG->CTLR = IWDG_KEY_CHANGE;
    IWDG->PSCR = prescaler;

	// Enable changes then set rst_val, limited to max value
	IWDG->CTLR = IWDG_KEY_CHANGE;
    IWDG->RLDR = rst_val & 0x0FFF;

	// Enable Watchdog
	IWDG->CTLR = IWDG_KEY_ENABLE;
}



/*** Systick Functions *******************************************************/
static const uint32_t SYSTICK_ONE_MILLISECOND = ((uint32_t)FUNCONF_SYSTEM_CORE_CLOCK / 1000);
void systick_init(void)
{
	// Reset any pre-existing configuration
	SysTick->CTLR = 0x0000;
	
	// Set the compare register to trigger once per millisecond
	SysTick->CMP = SYSTICK_ONE_MILLISECOND - 1;

	// Reset the Count Register, and the global millis counter to 0
	SysTick->CNT = 0x00000000;
	_systick_millis = 0x00000000;
	
	// Set the SysTick Configuration
	// NOTE: By not setting SYSTICK_CTLR_STRE, we maintain compatibility with
	// busywait delay funtions used by ch32fun.
	SysTick->CTLR |= SYSTICK_CTLR_STE   |  // Enable Counter
	                 SYSTICK_CTLR_STIE  |  // Enable Interrupts
	                 SYSTICK_CTLR_STCLK ;  // Set Clock Source to HCLK/1
	
	// Enable the SysTick IRQ
	NVIC_EnableIRQ(SysTicK_IRQn);
}


__attribute__((interrupt))
void SysTick_Handler(void)
{
	// Increment the Compare Register for the next trigger
	SysTick->CMP += SYSTICK_ONE_MILLISECOND;

	// Clear the trigger state for the next IRQ
	SysTick->SR = 0x00000000;

	// Increment the milliseconds count
	_systick_millis++;
}


/*** PWM Functions ***********************************************************/
void pwm_init(void)
{
	// NOTE: Uses TIM2 Channel 2 (PD3) and TIM2 Channel 3 (PC0)

	// Enable TIM2 Clock
	RCC->APB1PCENR |= RCC_APB1Periph_TIM2;

	// Enable the GPIO PORT C and PORT D Clocks
	RCC->APB2PCENR |= RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOC;

	// Alternate Function PushPull Outputs
	GPIOC->CFGLR &= ~(0x0F << 0);
	GPIOC->CFGLR |=  (GPIO_CNF_OUT_PP_AF | GPIO_Speed_10MHz) << 0;       // PC0
	GPIOD->CFGLR &= ~(0x0F << (3 * 4));
	GPIOD->CFGLR |=  (GPIO_CNF_OUT_PP_AF | GPIO_Speed_10MHz) << (3 * 4); // PD3

	// Reset TIM2, Inits all registers
	RCC->APB1PRSTR |=  RCC_APB1Periph_TIM2;
	RCC->APB1PRSTR &= ~RCC_APB1Periph_TIM2;

	// Set Prescaler to ~17KHz. More efficient switching
	TIM2->PSC = 0x000A;
	// Set PWM Max Value (Autoreload Value)
	TIM2->ATRLR = 254;

	// Set the Compare Capture Register for Channel 3
	// TIM2_OC3M = 0b111 - PWM Mode 2 - Enable Preload
	TIM2->CHCTLR1 |= TIM_OC2M_2 | TIM_OC2M_1 | TIM_OC2M_0 | TIM_OC2PE;
	TIM2->CHCTLR2 |= TIM_OC3M_2 | TIM_OC3M_1 | TIM_OC3M_0 | TIM_OC3PE;

	// Enable auto-reload
	TIM2->CTLR1 |= TIM_ARPE;

	// Enable channel output, polarity is ACTIVE_HIGH
	TIM2->CCER |= TIM_CC2E | TIM_CC2P;
	TIM2->CCER |= TIM_CC3E | TIM_CC3P;

	// Initialise Counter
	TIM2->SWEVGR |= TIM_UG;

	// Enable TIM2
	TIM2->CTLR1 |= TIM_CEN;
}
