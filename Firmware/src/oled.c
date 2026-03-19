/******************************************************************************
* I2C OLED Text and "Graphics" Handler, Buffer and manager library - customised
* for TCRHMAG
*
* ADBeta (c)
******************************************************************************/
#include "oled.h"
#include "lib_i2c.h"

#include <stddef.h>
#include <stdint.h>

// TODO:
#include <stdio.h>
#include <stdbool.h>
#include <string.h>


/*** Macro Functions *********************************************************/
#define OLED_WRITE(reg, buf, size)                     \
	do {                                               \
		if(i2c_write_reg(_oled_dev, (reg),             \
				        (buf), (size)) != I2C_OK)      \
			return OLED_I2C_FAILED;                    \
	} while(0)


/*** Static Variables ********************************************************/
/// @brief Pointer to the OLED I2C Device for later use - use init() to set
static const i2c_device_t *_oled_dev = NULL;


// Resets the OLED Column Position and sets the window from 0 to 127 px
static const uint8_t _col_arr[] = { 0x21, 0x00, 0x7F };
// Resets the OLED Page Position and sets the windows from 0 to 7 pages (64px)
static const uint8_t _page_arr[] = { 0x22, 0x00, 0x07 };



/// @brief Buffer for the whole OLED Display - 128x64 pixel - 128x8 2D Array
static uint8_t _oled_buffer[8][128] = {0x00};


/*** Function Definitons *****************************************************/
oled_err_t oled_init(const i2c_device_t *dev)
{
	if(dev == NULL) return OLED_INVALID_INPUT;

	// Set the static i2c device value to the given device
	_oled_dev = dev;

	// Magic init byte array for the OLED :)   (See Datasheet for info)
	const uint8_t init_arr[] = {
		0xAE, 0xD4, 0x80, 0xA8, 0x1F, 0xD3, 0x00, 0x40,	0x8D, 0x14, 
		0x20, 0x00, 0xA1, 0xC8, 0xDA, 0x12, 0x81, 0x01, 0xD9, 0xF1, 
		0xDB, 0x40, 0xA4, 0xA6, 0xAF
	};

	// Write the Initialisation Array to the Display
	OLED_WRITE(OLED_REG_COMMAND, init_arr,  sizeof(init_arr));
	
	return OLED_OK;
}



oled_err_t oled_update(void)
{
	if(_oled_dev == NULL) return OLED_UNSET_DEVICE;

	// Reset the Position Pointers and set the usable window for the display
	OLED_WRITE(OLED_REG_COMMAND, _col_arr,  sizeof(_col_arr));
	OLED_WRITE(OLED_REG_COMMAND, _page_arr, sizeof(_page_arr));

	// Write the OLED Display Data
	OLED_WRITE(OLED_REG_DISPLAY_START, (uint8_t *)_oled_buffer, sizeof(_oled_buffer));

	return OLED_OK;
}



void oled_loop()
{
	while(true)
	{
		static uint8_t fill = 0x00;
		
		memset((uint8_t *)_oled_buffer, fill, sizeof(_oled_buffer));
		// TODO: update hangs
		//oled_update();
		printf("fill = 0x%02X\n", fill);

		fill = ~fill;

		Delay_Ms(1000);
	}


};
