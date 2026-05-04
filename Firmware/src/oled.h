/******************************************************************************
* I2C OLED Text and "Graphics" Handler, Buffer and manager library - customised
* for TCRHMAG
* Really shitty design because I ran out of patience
*
* ADBeta (c)    04 May 2026    Ver 1.1
******************************************************************************/
#ifndef TCRHMAG_OLED_H
#define TCRHMAG_OLED_H

#include "lib_i2c.h"

#include <stdint.h>

/*** OLED Definitions ********************************************************/
#define OLED_REG_COMMAND              0x00
#define OLED_REG_DISPLAY_START        0x40
#define OLED_WIDTH                    128
#define OLED_PAGES                    8



/*** Typedefs and Structs ****************************************************/
typedef enum {
	OLED_OK                       = 0,
	OLED_UNSET_DEVICE,
	OLED_INVALID_INPUT,
	OLED_I2C_FAILED,


} oled_err_t;



/*** Hardware Function Declarations ******************************************/
/// @brief Initialises the OLED into the configuration desired by the library
/// @param Pointer to the I2C Device to use for the OLED
/// @return oled_err_t status code, OLED_OK if successful
oled_err_t oled_init(const i2c_device_t *dev);


/// @brief Sets the cursor of the OLED, so the next data sent is at the given
/// offet.
/// @param x, X position to set the cursor to (Pixels)
/// @param y, Y position to set the cursor to (Pages)
/// @return oled_err_t status code, OLED_OK if successful
oled_err_t oled_set_cursor(const uint8_t x, const uint8_t y);


/// @param Writes the whole framebuffer to the OLED
/// @param None
/// @return oled_err_t status code, OLED_OK if successful
oled_err_t oled_update(void);



/*** Drawing API Functions ***************************************************/
/// @brief Draws the Battery Information to the top of the display
/// @param vbat_ma, Battery Current in milliamps
/// @param vbat_mv, Battery voltage in millivolts
/// @param vbat_pc, Battery Percentage
/// @return None
oled_err_t oled_draw_battery_info(const uint16_t batt_mv,
	                              const uint16_t batt_ma, 
								  const uint16_t batt_pc);






/*** Grpahics Data ***********************************************************/

#endif
