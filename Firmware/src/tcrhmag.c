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
* Ver 0.2    07 Mar 2026
* (c) ADBeta 2026
******************************************************************************/
#include "ch32fun.h"
#include "thermistor_lut.h"
#include "lib_gpioctrl.h"
#include "lib_pid.h"
#include "opamp.h"


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

#define PWM_CHANNEL_LED               2
#define PWM_CHANNEL_HEATER            3


/*** Timing Values ***********************************************************/
#define MILLIS_LED_UPDATE            25
#define MILLIS_PID_TEMP_UPDATE       100
#define MILLIS_BATTERY_CHECK         250


/*** Global Variables ********************************************************/
// 1ms SysTick counter variable
volatile uint32_t g_systick_millis         = 0;


/// System Status and Settings ////////////////////////////////////////////////
// System //////
static bool       g_heater_enabled                          = false;
// Battery /////
static uint16_t   g_battery_voltage_mv;
static uint16_t   g_battery_current_ma;
// TODO:
#define           BATTERY_OVERCURRENT_SHUTDOWN_MA           7500
#define           BATTERY_UNDERVOLTAGE_WARNING_MV           1
#define           BATTERY_UNDERVOLTAGE_SHUTDOWN_MV          2
// Heater //////
static uint16_t   g_target_temperature;
static uint16_t   g_measured_temperature;



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


/// @breif Initialised TIM2 Channel 2 (PD3) and Channel 3 (PC0) to be 
/// PWM Output, active LOW.
/// Autoreload is set to 254, Capture mode is 0b111, PWM2.
/// @param none
/// @return none
static void pwm_init(void);


/// @breif Sets the Duty Cycle of the given PWM Channel. Max input is 255
/// @param ch, channel to change (Only 2 & 3 are valid)
/// @param duty, input duty cycle value
/// @return none
static void pwm_set_duty(const uint8_t ch, const uint32_t duty);


/// @brief Converts a given Thermistor Raw ADC Value to a Temperature (c)
/// using Linear Interpolation of the Pre-computed LUT
/// @param adc, Raw ADC Value of the Thermistor
/// @return Temperature in degrees Celcius
static uint16_t thermistor_adc_to_temp(const uint16_t adc);


/// @brief Updates the LEDs current PWM Value depending on if the system is
/// running, and other status events
/// @param fading, bool enable/disable value
/// @return None
static void update_led(const bool fading);




/*** Main ********************************************************************/
int main(void)
{
	SystemInit();

	/*** IO Initialisation *******************************/
	// Init the I2C Handler at 1MHz, then initialise the display
	//i2c_init(I2C_CLK_1MHZ);

	// Initialise PWM Channels. Turn off the Heater (CH3) and the LED on (CH2)
	pwm_init();
	pwm_set_duty(PWM_CHANNEL_LED, 0xFF);
	pwm_set_duty(PWM_CHANNEL_HEATER, 0x00);

	// Initiliase the ADC to use 24MHz clock, and Sample for 43 Clock Cycles
	gpio_init_adc(ADC_CLOCK_DIV_2, ADC_SAMPLE_CYCLES_43);

	// Initialise the OpAmp Pins - Use CHA1 +/-,  CHA2 +, and ADC Output
	gpio_set_mode(OPAMP_CHA1_NEG, INPUT_FLOATING);
	gpio_set_mode(OPAMP_CHA1_POS, INPUT_FLOATING);
	gpio_set_mode(OPAMP_CHA2_POS, INPUT_FLOATING);

	gpio_set_mode(THERM_ADC_PIN,  INPUT_ANALOG);
	gpio_set_mode(BSENS_ADC_PIN,  INPUT_ANALOG);
	gpio_set_mode(OPAMP_ADC_PIN,  INPUT_ANALOG);	



	// TODO:
	// Initialise the Internal OpAmp and set it to use CH1+ & CH1-
	gpio_init_opamp();
	gpio_set_opamp_inputs(GPIO_OPAMP_CH1_POS, GPIO_OPAMP_CH1_NEG);



	/*** Timers and Watchdog *****************************/
	// Initialise the SysTick to get millis() funcitonality
	systick_init();

	// TODO:
	// Initialise the IWDT to prevent a lockup - 250ms
	iwdg_init(0x04, 0x9B);



	// TODO:
	// Get user settings from flash
	/*** PID Controller **********************************/
	// Declare and initialise a PID Controller Struct for the Heater
	pid_ctrl_t heater_pid =
	{
		.Kp        = 1000,
		.Ki        = 5,
		.Kd        = 5,
		.scale     = 100,

		.out_min   = 0,
		.out_max   = 255
	};
	pid_init(&heater_pid);




	// TODO:
	g_heater_enabled = true;
	g_target_temperature = 200;


	uint32_t millis_prev_led_update        = 0;
	uint32_t millis_prev_pid_temp_update   = 0;
	uint32_t millis_prev_battery_check     = 0;


	while(true)
	{
		/// LED Update //////////////////////////////////////////////////////////////
		if(g_systick_millis - millis_prev_led_update > MILLIS_LED_UPDATE)
		{
			update_led(g_heater_enabled);
			millis_prev_led_update = g_systick_millis;
		}


		/// PID and Temp Update /////////////////////////////////////////////////////
		if(g_systick_millis - millis_prev_pid_temp_update > MILLIS_PID_TEMP_UPDATE)
		{
			static uint16_t   thermistor_adc;
			static int32_t    heater_pwm;

			// Get the RAW ADC Value from the Thermistor, and convert it to an
			// interpolated Temperature
			thermistor_adc = gpio_analog_read(THERM_ADC_CH);
			g_measured_temperature = thermistor_adc_to_temp(thermistor_adc);
		
			// Calculate the PWM Value needed for the heater using PID - regardless
			// of enabled state to keep the PID loop up-to-date
			heater_pwm = pid_calculate(&heater_pid, g_measured_temperature, g_target_temperature);
			
			// If the heater is disabled, set PWM value to 0 (OFF)
			if(!g_heater_enabled) heater_pwm = 0x00;
			pwm_set_duty(PWM_CHANNEL_HEATER, (uint8_t)heater_pwm);
		
		
			printf("temp: %d\tpwm: %lu\n", g_measured_temperature, heater_pwm);

			millis_prev_pid_temp_update = g_systick_millis;
		}


		/// Battery Checking ////////////////////////////////////////////////////////
		if(g_systick_millis - millis_prev_battery_check > MILLIS_BATTERY_CHECK)
		{
			// TODO: 
			// Get Battery Voltage and Current
			// Re-calibrate the opamp ever x number of cycles, or at voltage delta points??
			millis_prev_battery_check = g_systick_millis;
		}


		// Keep the Watchdog fed and happy :)
		iwdg_feed();
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
{ IWDG->CTLR = 0xAAAA; }




static const uint32_t SYSTICK_ONE_MILLISECOND = ((uint32_t)FUNCONF_SYSTEM_CORE_CLOCK / 1000);
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
	// NOTE: Uses TIM2 Channel 2 (PD3) and TIM2 Channel 3 (PC0)

	// Enable TIM2 Clock
	RCC->APB1PCENR |= RCC_APB1Periph_TIM2;

	// Set GPIO OUTPUT 10MHz, Aleternate Function (Multiplex)
	gpio_set_mode(GPIO_PC0, OUTPUT_10MHZ_PP | OUTPUT_PP_AF);
	gpio_set_mode(GPIO_PD3, OUTPUT_10MHZ_PP | OUTPUT_PP_AF);

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


static  void pwm_set_duty(const uint8_t ch, const uint32_t duty)
{
	switch(ch)
	{
		case 2:
			TIM2->CH2CVR = duty;
			break;
		case 3:
			TIM2->CH3CVR = duty;
			break;
		default:
			break;
	}
}


static uint16_t thermistor_adc_to_temp(const uint16_t adc)
{
	uint16_t adc0 = 0, adc1 = 0, temp0 = 0, temp1 = 0, output = 0;
	const uint16_t therm_max_index = THERMISTOR_PAIR_ENTRIES - 1;
	
	// Cap the Upper and Lower Bounds
	if(adc >= thermistor_pair[0].adc)
		return thermistor_pair[0].temp_c;
	
	if(adc <= thermistor_pair[therm_max_index].adc)
		return thermistor_pair[therm_max_index].temp_c;


	// Find the LUT ADC Range
	for(uint16_t ti = 0; ti < therm_max_index; ti++)
	{
		// Get the Current and Next ADC Values in the pair table.
		adc0  = thermistor_pair[ti].adc;
		adc1  = thermistor_pair[ti + 1].adc;

		// ADC Values Decrease as temperature rises. Check if adc input is within
		// the current check range
		if(adc <= adc0 && adc >= adc1)
		{
			// Get Current and Next Temp values in the pair table
			temp0 = thermistor_pair[ti].temp_c;
			temp1 = thermistor_pair[ti + 1].temp_c;

			// Linear Interpolation
			// ADC Decreases with Temperature Rise:
			// delta_adc   =   adc0 - adc
			// span_adc    =   adc0 - adc1
			// delta_temp  =   temp1 - temp0
			//
			// x = temp0 + (delta_adc * delta_temp) / span_adc
			output = temp0 + (uint32_t)(adc0 - adc) * (temp1 - temp0) / (adc0 - adc1);
			break;
		}
	}

	// 0 if something has gone wrong, the linear interptreted temp if not
	return output;
}


static void update_led(const bool fading)
{
	#define LED_MIN_VAL   40
	#define LED_MAX_VAL   255

	static int16_t led_val = LED_MIN_VAL;
	static int8_t  led_inc = 5;
	
	// If the system is running, Fade the LED in and out steadily
	if(fading)
	{
		led_val += led_inc;

		// Flip to decrimenting if maximum is hit
		if(led_val >= LED_MAX_VAL) { led_val = LED_MAX_VAL; led_inc = -led_inc; }

		// Flip to incrimenting if the minimum is hit
		if(led_val <= LED_MIN_VAL) { led_val = LED_MIN_VAL; led_inc = -led_inc; }

	// If the system isn't running, set the LED to maximum value
	} else { led_val = LED_MAX_VAL; }


	// Set the LED PWM to the calculated value
	pwm_set_duty(PWM_CHANNEL_LED, (uint8_t)led_val);
}
