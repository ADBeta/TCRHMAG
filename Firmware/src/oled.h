/******************************************************************************
* I2C OLED Text and "Graphics" Handler, Buffer and manager library - customised
* for TCRHMAG
*
* ADBeta (c)    19 Mar 2026    Ver 1.0
******************************************************************************/
#ifndef TCRHMAG_OLED_H
#define TCRHMAG_OLED_H

#include "lib_i2c.h"

#include <stdint.h>

/*** OLED Hardware Registers *************************************************/
#define OLED_REG_COMMAND              0x00
#define OLED_REG_DISPLAY_START        0x40

/*** Typedefs and Structs ****************************************************/
typedef enum {
	OLED_OK                       = 0,
	OLED_UNSET_DEVICE,
	OLED_INVALID_INPUT,
	OLED_I2C_FAILED,


} oled_err_t;



/*** Function Declarations ****************************************************/
/// @brief Initialises the OLED into the configuration desired by the library
/// @param Pointer to the I2C Device to use for the OLED
/// @return oled_err_t status code, OLED_OK if successful
oled_err_t oled_init(const i2c_device_t *dev);


/// @brief Writes the OLED Buffer out to the OLED in one transfer
/// @param None
/// @return oled_err_t status code, OLED_OK if successful
oled_err_t oled_update(void);


/// @brief Draws the Battery Information to the top of the display
/// @param vbat_mv, Battery voltage in millivolts
/// @param vbat_pc, Battery Percentage
/// @return None
oled_err_t oled_draw_battery_info(const uint16_t vbat_mv, 
								  const uint16_t vbat_pc);




/*** Font Data ***************************************************************/
/// @brief Small Inverted Limited Number/Symbol font for vbatt readout
const uint8_t small_inv_font_data[][] = {
	{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},   // ' '
	{0xB9, 0x99, 0xCF, 0xE7, 0xF3, 0x99, 0x9D, 0xFF},   // '%'
	{0xFF, 0xFF, 0x9F, 0x9F, 0xFF, 0xFF, 0xFF, 0xFF},   // '.'
	{0xC1, 0x80, 0x8E, 0xA6, 0xB2, 0x80, 0xC1, 0xFF},   // '0'
	{0xBF, 0xBD, 0x80, 0x80, 0xBF, 0xBF, 0xFF, 0xFF},   // '1'
	{0x9D, 0x8C, 0xA6, 0xB6, 0x90, 0x99, 0xFF, 0xFF},   // '2'
	{0xDD, 0x9C, 0xB6, 0xB6, 0x80, 0xC9, 0xFF, 0xFF},   // '3'
	{0xE7, 0xE3, 0xE9, 0xAC, 0x80, 0x80, 0xAF, 0xFF},   // '4'
	{0xD8, 0x98, 0xBA, 0xBA, 0x82, 0xC6, 0xFF, 0xFF},   // '5'
	{0xC3, 0x81, 0xB4, 0xB6, 0x86, 0xCF, 0xFF, 0xFF},   // '6'
	{0xFC, 0xFC, 0x8E, 0x86, 0xF0, 0xF8, 0xFF, 0xFF},   // '7'
	{0xC9, 0x80, 0xB6, 0xB6, 0x80, 0xC9, 0xFF, 0xFF},   // '8'
	{0xF9, 0xB0, 0xB6, 0x96, 0xC0, 0xE1, 0xFF, 0xFF},   // '9'
	{0xE3, 0xC3, 0x9F, 0x9F, 0xC3, 0xE3, 0xFF, 0xFF}    // 'v'
};

const char small_inv_font_lut[] = {" %.0123456789v"};



/*** Grpahics Data ***********************************************************/

#endif
