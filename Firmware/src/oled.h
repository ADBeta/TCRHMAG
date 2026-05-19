/******************************************************************************
* I2C OLED Text and "Graphics" Handler, Buffer and manager library - customised
* for TCRHMAG
* Really shitty design because I ran out of patience
*
* ADBeta (c)    16 May 2026    Ver 2.0
******************************************************************************/
#ifndef TCRHMAG_OLED_H
#define TCRHMAG_OLED_H

#include "lib_i2c.h"

#include <stdbool.h>
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
/// @brief Draws the Boot Screen onto the OLED
/// @param None
/// @return None
void oled_draw_boot_screen(void);


/// @brief Draws a screen to let the user know there was an error
/// @param None
/// @return None
void oled_draw_error_screen(void);


/// @brief Clears the whole display
/// @param None
/// @return None
void oled_clear_display(void);


/// @brief Clears the Battery Info bar to an empty state
/// @param None
/// @return None
void oled_clear_battery_info(void);


/// @brief Draws the Current Draw from the Battery to the OLED
/// @param batt_ma, Battery Current in milliamps
/// @return None
void oled_draw_battery_current(const uint16_t batt_ma);


/// @brief Draws the Voltage of the Battery to the OLED
/// @param batt_mv, Battery Voltage in millivolts
/// @return None
void oled_draw_battery_voltage(const uint16_t batt_mv);


/// @brief Draws the Battery Percentage to the OLED
/// @param batt_perc, Battery Percentage. 0-100, specifically 
/// 0%-25%, 25%-50%, 50%-75%, 75%-100%
/// @return None
void oled_draw_battery_percent(const uint8_t batt_perc);


/// @brief Draws the Temperature to the OLED
/// @param target, Target temperature value
/// @param actual, Actual temperature value
/// @return None
void oled_draw_temperature(const uint16_t target, const uint16_t actual);


/// @brief Draws an icon to let the user know if the Heater is on or off
/// @param heater_on, boolean if the heater is enabled or not
/// @return None
void oled_draw_heater_state(const bool heater_on);


/*** Grpahics Data ***********************************************************/

#endif
