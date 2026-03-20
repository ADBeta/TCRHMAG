/******************************************************************************
* PID Thermally Controlled Ryobi Hot Melt Adhesive Gun (HMAG)
* Uses OLED Display to show the Battery and Temperature Status, with
* Rotary Encoder for UI Inputs 
* See the GitHub for full documentation: https://github.com/ADBeta/TCRHMAG
*
*
* Pinout:
* PA1    OPAMP_CH1-
* PA2    OPAMP_CH1+
*
* PC0    PWM Out           (TIM2 CH3)
* PC1    I2C SDA
* PC2    I2C_SCL
* PC3    Rotary Encoder Switch
* PC4    Rotary Encoder A
* PC5    Rotary Encoder B
*
* PD3    LED Out           (TIM2 CH2)
* PD4    OPAMP_ADC         (ADC7)
* PD5    Thermistor ADC    (ADC5)
* PD6    Battery Sense     (ADC6)
* PD7    OPAMP_CH2+
*
*
* Ver 0.6    19 Mar 2026
* (c) ADBeta 2026
******************************************************************************/
#include "ch32fun.h"
#include "thermistor_lut.h"
#include "lib_gpioctrl.h"
#include "lib_i2c.h"
#include "lib_pid.h"
#include "battery.h"
#include "oled.h"


#include <stdio.h>
#include <string.h>
#include <stdbool.h>


/*** Macro Functons **********************************************************/
#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))


/*** Pin Definitions *********************************************************/
#define OPAMP_CHA1_NEG                GPIO_PA1
#define OPAMP_CHA1_POS                GPIO_PA2
#define OPAMP_CHA2_POS                GPIO_PD7
#define PWM_OUT                       GPIO_PC0

#define ROTENC_SW_PIN                 GPIO_PC3
#define ROTENC_A_PIN                  GPIO_PC4
#define ROTENC_B_PIN                  GPIO_PC5

#define THERM_ADC_PIN                 GPIO_A5
#define THERM_ADC_CH                  GPIO_ADC_A5
#define BSENS_ADC_PIN                 GPIO_A6
//#define BSENS_ADC_CH                GPIO_ADC_A6
#define OPAMP_ADC_PIN                 GPIO_A7
//#define OPAMP_ADC_CH                GPIO_ADC_A7

#define PWM_CHANNEL_LED               2
#define PWM_CHANNEL_HEATER            3



/*** Timing Values ***********************************************************/
#define MILLIS_LED_UPDATE            25
#define MILLIS_USER_INPUT_UPDATE     50
#define MILLIS_PID_TEMP_UPDATE       100
#define MILLIS_BATTERY_CHECK         250
#define MILLIS_DISPLAY_UPDATE        200



/*** Typedefs and Structures *************************************************/
typedef enum {
	DISPLAY_MODE_BOOTUP           = 0,
	DISPLAY_MODE_CHANGE_TEMP,
	DISPLAY_MODE_CHANGE_KP,
	DISPLAY_MODE_CHANGE_KI,
	DISPLAY_MODE_CHANGE_KD,
	DISPLAY_MODE_LOCKOUT
} display_mode_e;



/*** Global Variables ********************************************************/
// 1ms SysTick counter variable
volatile uint32_t g_systick_millis         = 0;


/// System Status and Settings ////////////////////////////////////////////////
// System //////
static display_mode_e      g_display_mode                        = DISPLAY_MODE_BOOTUP;
static bool                g_heater_enabled                      = false;
static bool                g_control_lockout                     = false;
static volatile int16_t    g_rotary_encoder_clicks               = 0;
// Battery /////
#define                    BATTERY_OVERCURRENT_SHUTDOWN_MA       5500
#define                    BATTERY_UNDERVOLTAGE_WARNING_MV       16000
#define                    BATTERY_UNDERVOLTAGE_SHUTDOWN_MV      15200
static uint16_t            g_battery_voltage_mv                  = 0;
static uint16_t            g_battery_current_ma                  = 0;
static uint8_t             g_battery_percentage                  = 0;
// Heater //////
#define                    SETTING_TEMPERATURE_MINIMUM           50
#define                    SETTING_TEMPERATURE_MAXIMUM           220
#define                    SETTING_TEMPERATURE_INCRIMENT         5
static int16_t             g_target_temperature                  = SETTING_TEMPERATURE_MINIMUM;
static int16_t             g_measured_temperature                = 0;


/// @brief Gray Code Lookup Table for the Rotary Encoder
static const int8_t rotenc_table[16] = 
{
	 0,  -1,   1,   0,
	 1,   0,   0,  -1,
	-1,   0,   0,   1,
	 0,   1,  -1,   0
};



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


/// @brief EXTI Pin Change Interrupt - Handles the Rotary Encoder stepping
/// @param None
/// @return None
__attribute__((interrupt)) 
void EXTI7_0_IRQHandler(void);


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





uint8_t _oled_buffer[8][128] = {0x00};





/*** Main ********************************************************************/
int main(void)
{
	SystemInit();

	/*** IO Initialisation *******************************/
	// I2C Setup for the OLED
	i2c_device_t i2c_oled = {
		.clkr = I2C_CLK_1MHZ,
		.type = I2C_ADDR_7BIT,
		.addr = 0x3C,
		.regb = 1,
		.tout = 2000,
	};
	i2c_init(&i2c_oled);

	// Initialise the OLED, then display the BOOTING Screen
	oled_init(&i2c_oled);


	// Initialise PWM Channels. Turn off the Heater (CH3) and the LED on (CH2)
	pwm_init();
	pwm_set_duty(PWM_CHANNEL_LED,    0xFF);
	pwm_set_duty(PWM_CHANNEL_HEATER, 0x00);
	
	// Set the Rotary Encoder Pins to INPUT_PULLUP
	gpio_set_mode(ROTENC_SW_PIN, INPUT_PULLUP);
	gpio_set_mode(ROTENC_A_PIN,  INPUT_PULLUP);
	gpio_set_mode(ROTENC_B_PIN,  INPUT_PULLUP);

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
	// Print "Booting" or something to screen
	gpio_init_opamp();
	gpio_set_opamp_inputs(GPIO_OPAMP_CH1_POS, GPIO_OPAMP_CH1_NEG);
	opamp_calibrate();


	/*** Pin Change Interrupts (Rotary Encoder) **********/
	// Enable AFIO and CH4 and CH5 on PORTC
	RCC->APB2PCENR |= RCC_APB2Periph_AFIO;
	AFIO->EXTICR    = AFIO_EXTICR_EXTI4_PC | AFIO_EXTICR_EXTI5_PC;
	
	// Enable IRQ on Channel 4 & 5, Rising or Falling Edge
	EXTI->INTENR    = EXTI_INTENR_MR4 | EXTI_INTENR_MR5;
	EXTI->RTENR     = EXTI_RTENR_TR4  | EXTI_RTENR_TR5;
	EXTI->FTENR     = EXTI_FTENR_TR4  | EXTI_FTENR_TR5;

	// Clear IRQ Flags before enabling for safety
	EXTI->INTFR = 0x00000030;
	// Enable the EXTI0-7 Interrupt Group in NVIC
	NVIC_EnableIRQ(EXTI7_0_IRQn);


	/*** Timers and Watchdog *****************************/
	// Initialise the SysTick to get millis() funcitonality
	systick_init();

	// TODO:
	// Initialise the IWDT to prevent a lockup - 250ms
	//iwdg_init(0x04, 0x9B);



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
	//g_heater_enabled = true;


	uint32_t millis_prev_led_update        = 0;
	uint32_t millis_prev_user_input_update = 0;
	uint32_t millis_prev_pid_temp_update   = 0;
	uint32_t millis_prev_battery_check     = 0;
	uint32_t millis_prev_display_update    = 0;





	

	while(true)
	{
		/// LED Update //////////////////////////////////////////////////////////////
		if(g_systick_millis - millis_prev_led_update > MILLIS_LED_UPDATE)
		{
			update_led(g_heater_enabled);
			millis_prev_led_update = g_systick_millis;
		}


		/// User Control Input Checks ///////////////////////////////////////////////
		if(g_systick_millis - millis_prev_user_input_update > MILLIS_USER_INPUT_UPDATE)
		{

			//if(g_display_mode == DISPLAY_MODE_CHANGE_TEMP)
			//{
				// Incriment / Decriment the Target Temp by the current clicks
				// Limit to min and max values, then reset clicks
				int16_t inc = g_rotary_encoder_clicks * SETTING_TEMPERATURE_INCRIMENT;
				g_target_temperature = CLAMP(g_target_temperature + inc, 
								             SETTING_TEMPERATURE_MINIMUM, 
								             SETTING_TEMPERATURE_MAXIMUM);
				g_rotary_encoder_clicks = 0;
				
			//}

			millis_prev_user_input_update = g_systick_millis;
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

			millis_prev_pid_temp_update = g_systick_millis;
		}


		/// Battery Checking ////////////////////////////////////////////////////////
		if(g_systick_millis - millis_prev_battery_check > MILLIS_BATTERY_CHECK)
		{
			uint16_t system_mv = gpio_read_system_mv();

			g_battery_voltage_mv = battery_read_mv(system_mv);
			g_battery_percentage = battery_calc_battery_percent(g_battery_voltage_mv);
			g_battery_current_ma = battery_read_average_ma(system_mv);
			

			// Stop the Heater Driver if the Battery Voltage drops below the shutdown
			// threshold value - or if the current is higher than the shutdown threshold
			// And disable the user controls so it cannot be renabled until reboot
			if(  (g_battery_voltage_mv <= BATTERY_UNDERVOLTAGE_SHUTDOWN_MV) ||
			     (g_battery_current_ma >= BATTERY_OVERCURRENT_SHUTDOWN_MA)  )
			{
				g_heater_enabled   = false;
				g_control_lockout  = true;
				g_display_mode     = DISPLAY_MODE_LOCKOUT;
			}

			millis_prev_battery_check = g_systick_millis;
		}


		/// Display Update and Redraw ///////////////////////////////////////////////
		if(g_systick_millis - millis_prev_display_update > MILLIS_DISPLAY_UPDATE)
		{
		
			oled_draw_battery_info(g_battery_voltage_mv, g_battery_percentage);
			oled_update();
			
			millis_prev_display_update = g_systick_millis;
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


__attribute__((interrupt))
void EXTI7_0_IRQHandler(void)
{
	// Trigger on CH4 and CH5
	if((EXTI->INTFR & 0x00000010) || (EXTI->INTFR & 0x00000020))
	{
		// Read the PORT Input Register to get the Pinm Bits
		uint32_t portc_read = GPIOC->INDR;

		// Clear whichever flag was hit
		if(EXTI->INTFR & 0x10) EXTI->INTFR = 0x10;
		if(EXTI->INTFR & 0x20) EXTI->INTFR = 0x20;

		// Accumulate transitions per detend (usually 4) 
		static int8_t accumulator     = 0;
		
		// Read A and B Pins (PC4 & PC5), use their state as bits in a bitmask. [AB]
		static int8_t rotenc_prev_bitmask = 0b11;
		       int8_t rotenc_curr_bitmask = (((portc_read >> 4) & 1) << 1) | ((portc_read >> 5) & 1);

		// Convert the bitmasks into a step value using the Gray Code Table
		int8_t step = rotenc_table[ (rotenc_prev_bitmask << 2) | rotenc_curr_bitmask ];
		accumulator += step;

		// If the Accumulator has incrimented or decrimented by the number of clicks
		// per indent (4), incriment or decriment the global value
		     if(accumulator >=  4) { g_rotary_encoder_clicks++; accumulator -= 4; }
		else if(accumulator <= -4) { g_rotary_encoder_clicks--; accumulator += 4; }
	
		// Update the prev bitmask for the next interrupt
		rotenc_prev_bitmask = rotenc_curr_bitmask;
	}
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
