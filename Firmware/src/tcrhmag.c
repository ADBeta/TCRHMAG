/******************************************************************************
*
*
*
*
* Pinout:
* PA1    OPAMP_CH1-
* PA2    OPAMP_CH1+
*
* PC0    PWM Out           (TIM2 CH3)
* PC1    I2C SDA
* PC2    I2C_SCL
*
* PD3    LED Out           (TIM2 CH2)
* PD4    OPAMP_ADC         (ADC7)
* PD5    Thermistor ADC    (ADC5)
* PD6    Battery Sense     (ADC6)
* PD7    OPAMP_CH2+
*
*
*
*
* Ver 0.1    07 Feb 2026
* (c) ADBeta 2026
******************************************************************************/
#include "ch32fun.h"
#include "lib_gpioctrl.h"
#include "opamp.h"
#include "thermistor_lut.h"

#include "stdio.h"
#include "string.h"
#include "stdbool.h"



/*** Pin Definitions *********************************************************/
#define OPAMP_CHA1_NEG                GPIO_PA1
#define OPAMP_CHA1_POS                GPIO_PA2
#define OPAMP_CHA2_POS                GPIO_PD7
#define PWM_OUT                       GPIO_PC0

#define THERM_ADC_PIN                 GPIO_A5
#define THERM_ADC_CH                  GPIO_ADC_A5
#define BSENS_ADC_PIN                 GPIO_A6
#define BSENS_ADC_CH                  GPIO_ADC_A6
#define OPAMP_ADC_PIN                 GPIO_A7
#define OPAMP_ADC_CH                  GPIO_ADC_A7



/*** Value Definitions *******************************************************/
#define SYSTICK_ONE_MILLISECOND ((uint32_t)FUNCONF_SYSTEM_CORE_CLOCK / 1000)



/*** Global Variables ********************************************************/
// 1ms SysTick counter variable
volatile uint32_t g_systick_millis         = 0;



/*** Function Declarations ***************************************************/
/// @brief Initialise the SysTick interrupt to incriment every 1 millisecond
/// @param None
/// @return None
static void systick_init(void);


/// @brief SysTick IRQ
/// @param none
/// @return none
__attribute__((interrupt))
void SysTick_Handler(void);


/// @brief Initialises the watchdog with prescaler and reset value
/// @param prescaler, Input CLK divider prescaler, 0x00 = 4x
/// @param rst_val, value to reset when reached
/// @return None
static void iwdg_init(const uint8_t prescaler, const uint16_t rst_val);


/// @brief Resets the watchdog timer
/// @param None
/// @return None
__attribute__((always_inline))
static inline void iwdg_feed(void);


/// @breif Initialised TIM2 Channel 3 (C0) To be a PWM Output, active LOW.
/// Autoreload is set to 254, Capture mode is 0b111, PWM2.
/// @param none
/// @return none
static void pwm_init(void);


/// @breif Sets the Duty Cycle of PWM output (C0). Max input is 255
/// @param duty, input duty cycle value
/// @return none
__attribute__((always_inline))
static inline void pwm_set_duty(const uint32_t duty);



/*** Main ********************************************************************/
int main(void)
{
	SystemInit();

	/*** IO Initialisation *******************************/
	// Init the I2C Handler at 1MHz, then initialise the display
	//i2c_init(I2C_CLK_1MHZ);

	// Initialise PWM and turn the output off
	pwm_init();
	pwm_set_duty(0x00);

	// Initiliase the ADC to use 24MHz clock, and Sample for 43 Clock Cycles
	gpio_init_adc(ADC_CLOCK_DIV_2, ADC_SAMPLE_CYCLES_43);

	// Initialise the OpAmp Pins - Use CHA1 +/-,  CHA2 +, and ADC Output
	gpio_set_mode(OPAMP_CHA1_NEG, INPUT_FLOATING);
	gpio_set_mode(OPAMP_CHA1_POS, INPUT_FLOATING);
	gpio_set_mode(OPAMP_CHA2_POS, INPUT_FLOATING);

	gpio_set_mode(THERM_ADC_PIN,  INPUT_ANALOG);
	gpio_set_mode(BSENS_ADC_PIN,  INPUT_ANALOG);
	gpio_set_mode(OPAMP_ADC_PIN,  INPUT_ANALOG);	



	// Initialise the Internal OpAmp and set it to use CH1+ & CH1-
	gpio_init_opamp();
	gpio_set_opamp_inputs(GPIO_OPAMP_CH1_POS, GPIO_OPAMP_CH1_NEG);



	/*** Timers and Watchdog *****************************/
	// Initialise the SysTick to get millis() funcitonality
	systick_init();

	// TODO:
	// Initialise the IWDT to prevent a lockup
	//iwdg_init(0x00, 0x04E2);


	while(true)
	{
		printf("%d\n", gpio_analog_read(THERM_ADC_CH));
		Delay_Ms(500);
	}

	return 0;
}





/*** Function Definitions ****************************************************/
static void iwdg_init(const uint8_t prescaler, const uint16_t rst_val)
{
	// Enable changes then set prescaler
	IWDG->CTLR = 0x5555;
    IWDG->PSCR = prescaler;

	// Enable changes then set rst_val, limited to max value
	IWDG->CTLR = 0x5555;
    IWDG->RLDR = rst_val & 0x0FFF;

	// Enable Watchdog
	IWDG->CTLR = 0xCCCC;
}


__attribute__((always_inline))
static inline void iwdg_feed(void)
{
	IWDG->CTLR = 0xAAAA;
}


static void systick_init(void)
{
	// Reset any pre-existing configuration
	SysTick->CTLR = 0x0000;
	
	// Set the compare register to trigger once per millisecond
	SysTick->CMP = SYSTICK_ONE_MILLISECOND - 1;

	// Reset the Count Register, and the global millis counter to 0
	SysTick->CNT = 0x00000000;
	g_systick_millis = 0x00000000;
	
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
	g_systick_millis++;
}


static void pwm_init(void)
{
	// NOTE: Uses TIM2 Channel 3 (PC0) as the PWM Output pin

	// Enable TIM2 Clock
	RCC->APB1PCENR |= RCC_APB1Periph_TIM2;

	// Set GPIO OUTPUT 10MHz, Aleternate Function (Multiplex)
	gpio_set_mode(GPIO_PC0, OUTPUT_10MHZ_PP | OUTPUT_PP_AF);

	// Reset TIM2, Inits all registers
	RCC->APB1PRSTR |= RCC_APB1Periph_TIM2;
	RCC->APB1PRSTR &= ~RCC_APB1Periph_TIM2;

	// Set Prescaler to ~17KHz. More efficient switching
	TIM2->PSC = 0x000A;
	// Set PWM Max Value (Autoreload Value)
	TIM2->ATRLR = 254;

	// Set the Compare Capture Register for Channel 3
	// TIM2_OC3M = 0b111 - PWM Mode 2 - Enable Preload
	TIM2->CHCTLR2 |= TIM_OC3M_2 | TIM_OC3M_1 | TIM_OC3M_0 | TIM_OC3PE;

	// Enable auto-reload
	TIM2->CTLR1 |= TIM_ARPE;

	// Enable channel output, polarity is ACTIVE_HIGH
	TIM2->CCER |= TIM_CC3E | TIM_CC3P;

	// Initialise Counter
	TIM2->SWEVGR |= TIM_UG;

	// Enable TIM2
	TIM2->CTLR1 |= TIM_CEN;
}


__attribute__((always_inline))
static inline void pwm_set_duty(const uint32_t duty)
{
	TIM2->CH3CVR = duty;
}
