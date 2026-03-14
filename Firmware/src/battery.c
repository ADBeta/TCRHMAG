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
static int16_t _opamp_calibration_gain;


#define AVG_SAMPLES    8
#define AVG_DIV_SHIFT  3
static uint16_t _average_current_buffer[AVG_SAMPLES] = {0x00};



/*** Function Definitions ****************************************************/
void opamp_calibrate(void)
{
	// Measure the offset of the OpAmp at 0 Input (While PWM Off)
	Delay_Ms(500);
	_opamp_calibration_offset = gpio_analog_read(OPAMP_ADC_CH);

	// Use Channel 2 of the OpAmp - this is a resistor divider to create
	// VCC/32 - The OpAmp Gain of 16 makes this effectively VCC/2
	// Compare the value output to the expected output (1023/2) to get an
	// offset
	
	// Switch to CH2 Positive Input and wait for it to stabilise
	gpio_set_opamp_inputs(GPIO_OPAMP_CH2_POS, GPIO_OPAMP_CH1_NEG);
	Delay_Ms(500);

	// Calculate the delta between the expected value and the read value
	int32_t delta = gpio_analog_read(OPAMP_ADC_CH) - _opamp_calibration_offset;
	if(delta == 0) delta = 1;
	
	_opamp_calibration_gain = ((int32_t)OPAMP_CAL_EXPECTED << 10) / delta;

	printf("offset: %d\tgain: %d\n", _opamp_calibration_offset, _opamp_calibration_gain);

	// Switch back to CH1 Positive and wait for it to settle
	gpio_set_opamp_inputs(GPIO_OPAMP_CH1_POS, GPIO_OPAMP_CH1_NEG);
	Delay_Ms(500);
}



uint16_t opamp_read_rsense_ma(const uint16_t system_mv)
{
	// Get the OpAmp RAW ADC Value, corrected the offset and apply gain correction
	int16_t opamp_calibrated_adc = 
		((gpio_analog_read(OPAMP_ADC_CH) - _opamp_calibration_offset) 
		* _opamp_calibration_gain) >> 10;

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


uint16_t battery_read_average_ma(const uint16_t system_mv)
{
	static uint32_t rolling_sum = 0;
	static uint8_t average_index = 0;

	// Subtract the oldest sample from the sum
	rolling_sum -= _average_current_buffer[average_index];

	// Read a new sample into the buffer and add it to the sum
	_average_current_buffer[average_index] = opamp_read_rsense_ma(system_mv);
	rolling_sum += _average_current_buffer[average_index];
	
	// Incriment the Buffer Index, wrap back to the start of the buffer
	average_index = (average_index + 1) & (AVG_SAMPLES - 1);

	// Return the sum (Power of 2 Buffer so shifting works)
	return (uint16_t)(rolling_sum >> AVG_DIV_SHIFT);
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


uint8_t battery_calc_battery_percent(const uint16_t vbatt)
{
	if(vbatt >= BATTERY_FULL_MV)    return 100;
	if(vbatt <= BATTERY_EMPTY_MV)   return 0;

	uint32_t diff_scaled = (vbatt - BATTERY_EMPTY_MV) * 100;
	return (diff_scaled / (BATTERY_FULL_MV - BATTERY_EMPTY_MV));
}
