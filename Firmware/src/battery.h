/******************************************************************************
* MCU ADC and OpAmp control header, for calibration and current reading from the 
* RSENSE circuit, and the VBATT Measurment on TCRHMAG
*
* ADBeta (c)    10 Mar 2026    Ver 1.0
******************************************************************************/
#include <stdint.h>


#ifndef BATTERY_H
#define BATTERY_H

#define BSENS_ADC_CH                GPIO_ADC_A6
#define OPAMP_ADC_CH                GPIO_ADC_A7


#define ADC_MAXIMUM           1023   // Maximum ADC Value
#define ADC_REF_MV            1200   // Internal VREF Voltage (mV)

#define OPAMP_GAIN_SHIFT      4      // OpAmp Gain Divider (/16 == >> 4)
#define OPAMP_SENSE_MILLOHM   37     // Current Sense Resistor (mOhm)
#define OPAMP_CAL_EXPECTED    512    // Expected OpAmp Cal value (1023/32) * 16


/*** Function Declarations ***************************************************/
/// @brief Calibrates the OpAmp to mitigate Input & Gain offsets
/// @param None
/// @return None
void opamp_calibrate(void);


/// @brief Reads the output of the OpAmp RSENSE Circuit and converts it to
/// mA values
/// @param system_mv, The current System Voltage in millivolts
/// @return milliamps being measured by the RSENSE Circuit
uint16_t opamp_read_rsense_ma(const uint16_t system_mv);


/// @brief Reads the Battery Voltage through a resistor divider and converts
/// it to millivolts
/// @param system_mv, the current System Voltage in millivolts
/// @return millivolts of the Battery
uint16_t battery_read_mv(const uint16_t system_mv);


#endif
