/******************************************************************************
* MCU ADC and OpAmp control header, for calibration and current reading from the 
* RSENSE circuit, and the VBATT Measurment on TCRHMAG
*
* ADBeta (c)    10 Mar 2026    Ver 1.0
******************************************************************************/
#include "battery.h"
#include "lib_gpioctrl.h"
#include "ch32fun.h"

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>


/*** Static Variables ********************************************************/
static int16_t _opamp_calibration_offset;




/*** Function Definitions ****************************************************/
void opamp_calibrate(void)
{
	// Use Channel 2 of the OpAmp - this is a resistor divider to create
	// VCC/32 - The OpAmp Gain of 16 makes this effectively VCC/2
	// Compare the value output to the expected output (1023/2) to get an
	// offset
	
	// Switch to CH2 Positive Input and wait for it to stabilise
	gpio_set_opamp_inputs(GPIO_OPAMP_CH2_POS, GPIO_OPAMP_CH1_NEG);
	Delay_Ms(500);

	// Do multiple readings and average them out
	// Calculate the delta between the expected value and the read value
	int32_t cal_avg = 0;
	for(uint8_t cycle = 0; cycle < 10; cycle++)
	{
		cal_avg += (OPAMP_CAL_EXPECTED - gpio_analog_read(OPAMP_ADC_CH));
		Delay_Ms(100);
	}

	// Set the Offset to the average value
	_opamp_calibration_offset = cal_avg / 10;


	printf("offset: %d\n\n", _opamp_calibration_offset);

	// Switch back to CH1 Positive and wait for it to settle
	gpio_set_opamp_inputs(GPIO_OPAMP_CH1_POS, GPIO_OPAMP_CH1_NEG);
	Delay_Ms(500);
}



uint16_t opamp_read_rsense_ma(const uint16_t system_mv)
{
	// Get the OpAmp RAW ADC Value, corrected by the calibration value
	int16_t opamp_calibrated_adc = 
		gpio_analog_read(OPAMP_ADC_CH) + _opamp_calibration_offset;
	
	// Limit Underflow or Overflows
	if(opamp_calibrated_adc < 0)     opamp_calibrated_adc = 0;
	if(opamp_calibrated_adc > 1023)  opamp_calibrated_adc = 1023;

	// Calculate the OpAmp Output mV raw, ignoring the OpAmp gain for now.
	uint16_t opamp_mv_raw = ((uint32_t)opamp_calibrated_adc * system_mv) / ADC_MAXIMUM;
	
	// Convert mV and mR to mA, then scale down by the gain -
	// Usually, calculate the milliamps via ohms law, scaled for intager maths
	uint16_t opamp_ma = ((opamp_mv_raw * 1000) / OPAMP_SENSE_MILLOHM) >> OPAMP_GAIN_SHIFT;

	return opamp_ma;
}


uint16_t battery_read_mv(const uint16_t system_mv)
{
	uint16_t battery_adc = gpio_analog_read(BSENS_ADC_CH);

	// The Voltage Divider is 33K || 8K2, to get the inverse of the divider ratio:
	//  R2      /     (R1    +    R2)
	// 8K2      /     (33K   +   8K2)
	// (Invert)
	// 41K2     /      8K2         = 5.024
	uint32_t scaled_mv = ((uint32_t)system_mv * 5024) / 1000;
	return ((uint32_t)battery_adc * scaled_mv) / ADC_MAXIMUM;
}
