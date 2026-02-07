/******************************************************************************
* lib_gpioctrl
* A runtime-capable GPIO Library, with Digital Read/Write and Analog Read
*
* See GitHub for details: https://github.com/ADBeta/CH32V003_lib_gpioctrl
*
* ADBeta (c) 2024-2026
******************************************************************************/
#include "lib_gpioctrl.h"
#include <stdint.h>


GPIO_PORT_REG_TypeDef *GPIO_PORT_MAP[4] = {
	GPIO_PORTA,
	NULL,
	GPIO_PORTC,
	GPIO_PORTD,
};


void gpio_set_mode(const GPIO_PIN pin, const GPIO_MODE mode)
{
	// TODO: Any pin over 8 needs to change CFGHR
	
	// Make array of uint8_t from [pin] enum. See definition for details
	uint8_t *byte = (uint8_t *)&pin;
	
	// Set the RCC Register to enable clock on the specified port
	GPIO_RCC->APB2PCENR |= (RCC_APB2PCENR_AFIO | (RCC_APB2PCENR_IOPxEN << byte[0]));

	// Clear then set the GPIO Config Register
	GPIO_PORT_MAP[ byte[0] ]->CFGLR &=        ~(0x0F  << (4 * byte[1]));
	GPIO_PORT_MAP[ byte[0] ]->CFGLR |=  (mode & 0x0F) << (4 * byte[1]);

	// If [mode] is INPUT_PULLUP or INPUT_PULLDOWN, set the [OUTDR] Register
	if(mode == INPUT_PULLUP || mode == INPUT_PULLDOWN)
		gpio_digital_write(pin, mode >> 4);
}


void gpio_init_opamp(void)
{
	GPIO_EXTEN->EXTEN_CTR |= GPIO_EXTEN_OPA_EN;
}


void gpio_set_opamp_inputs(const GPIO_OPAMP_CH_POS pos, 
						   const GPIO_OPAMP_CH_NEG neg)
{
	// Positive Input
	GPIO_EXTEN->EXTEN_CTR =
		(GPIO_EXTEN->EXTEN_CTR & ~GPIO_EXTEN_OPA_PSEL) |
		((pos == GPIO_OPAMP_CH2_POS) ? GPIO_EXTEN_OPA_PSEL : 0);

	// Negative Input
	GPIO_EXTEN->EXTEN_CTR =
		(GPIO_EXTEN->EXTEN_CTR & ~GPIO_EXTEN_OPA_NSEL) |
		((neg == GPIO_OPAMP_CH2_NEG) ? GPIO_EXTEN_OPA_NSEL : 0);
}


__attribute__((always_inline))
inline void gpio_digital_write(const GPIO_PIN pin, const GPIO_STATE state)
{
	// Make array of uint8_t from [pin] enum. See definition for details
	uint8_t *byte = (uint8_t *)&pin;

	uint32_t mask = 0x01 << byte[1];          // Shift by pin number
	if(state == GPIO_LOW) mask = mask << 16;  // Shift by 16 if LOW, to Reset

	GPIO_PORT_MAP[ byte[0] ]->BSHR = mask;
}


__attribute__((always_inline))
inline GPIO_STATE gpio_digital_read(const GPIO_PIN pin)
{
	// Make array of uint8_t from [pin] enum. See definition for details
	uint8_t *byte = (uint8_t *)&pin;

	// If the Input Reg has the wanted bit set, return HIGH
	if( (GPIO_PORT_MAP[ byte[0] ]->INDR & (0x01 << byte[1])) != 0x00 ) 
		return GPIO_HIGH;

	// else return LOW 
	return GPIO_LOW;
}


void gpio_init_adc(const ADC_CLOCK_DIV div, const ADC_SAMPLE_CYCLES cycles)
{
	// Enable the ADC clock
	GPIO_RCC->APB2PCENR |= RCC_APB2Periph_ADC1;

	// Set ACD Clock Divider
	GPIO_RCC->CFGR0 &= ~ADC_CLOCK_DIV_128;
	GPIO_RCC->CFGR0 |= div;

	// Reset the ADC, Inits all registers
	GPIO_RCC->APB2PRSTR |=  RCC_APB2Periph_ADC1;
	GPIO_RCC->APB2PRSTR &= ~RCC_APB2Periph_ADC1;
	
	// Set rule channel conversion to all 0, ready for read
	GPIO_ADC1->RSQR1 = 0;
	GPIO_ADC1->RSQR2 = 0;
	GPIO_ADC1->RSQR3 = 0;

	// Set the Sample Register(s) for all channels at once
	GPIO_ADC1->SAMPTR1 = cycles;
	GPIO_ADC1->SAMPTR2 = cycles;

	// Enable the ADC, and set the triggering to external
	GPIO_ADC1->CTLR2 |= ADC_ADON | ADC_EXTSEL;
	
	// Reset calibration, wait for it to finish
	GPIO_ADC1->CTLR2 |= ADC_RSTCAL;
	while(GPIO_ADC1->CTLR2 & ADC_RSTCAL);

	// Calibrate the ADC, want for it to finish
	GPIO_ADC1->CTLR2 |= ADC_CAL;
	while(GPIO_ADC1->CTLR2 & ADC_CAL);
}


__attribute__((always_inline))
inline uint16_t gpio_analog_read(const GPIO_ANALOG_CHANNEL chan)
{
	// Set rule channel conversion for single conversion on passed channel
	// GPIO_ADC1->RSQR1 = 0;
	// GPIO_ADC1->RSQR2 = 0;
	GPIO_ADC1->RSQR3 = (uint32_t)chan;

	GPIO_ADC1->CTLR2 |= ADC_SWSTART;
	while(!(GPIO_ADC1->STATR & ADC_EOC));

	// Get the resulting data from the ADC
	return GPIO_ADC1->RDATAR;
}


uint16_t gpio_read_system_mv(void)
{
	// Get the raw ADC Value for the VREF - shift by 10 (*1024),
	// then divide it by the ADC maximum.
	// This is the VREF Multiplier
	uint32_t vref_multiplier = ((uint32_t)GPIO_ADC_MAXIMUM << 10) / gpio_analog_read(GPIO_ADC_VREF);

	// The mV value is equal to the ADC_VREF Millivolts, multiplied by the VREF
	// Multiplier, then shifted by 10 (/1024) to scale back down to mV
	return (GPIO_ADC_REF_MV * vref_multiplier) >> 10;
}
