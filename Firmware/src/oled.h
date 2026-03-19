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



void oled_loop();



#endif
