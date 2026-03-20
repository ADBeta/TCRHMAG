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
//static uint8_t _oled_buffer[8][128] = {0x00};
extern uint8_t _oled_buffer[8][128];

#define OLED_PAGES  (sizeof(_oled_buffer)    / sizeof(_oled_buffer[0]))
#define OLED_WIDTH  (sizeof(_oled_buffer[0]) / sizeof(_oled_buffer[0][0]))



/*** Function Definitons *****************************************************/
oled_err_t oled_init(const i2c_device_t *dev)
{
	if(dev == NULL) return OLED_INVALID_INPUT;

	// Set the static i2c device value to the given device
	_oled_dev = dev;

	const uint8_t init_arr[] = {
		0xAE,             // Display OFF
		0xD5, 0xF0,       // Clock
		0xA8, 0x3F,       // Multiplex (64)
		0xD3, 0x00,       // Offset
		0x40,             // Start line
		0x8D, 0x14,       // Charge pump
		0x20, 0x00,       // Horizontal addressing
		0xA1,             // Segment remap
		0xC8,             // COM scan dec
		0xDA, 0x12,       // COM pins
		0x81, 0x7F,       // Contrast
		0xD9, 0x22,       // Pre-charge
		0xDB, 0x40,       // VCOM detect
		0xA4,             // Resume RAM
		0xA6,             // Normal display
		0xAF              // Display ON
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
	// Write all the display data
	for(int page = 0; page < OLED_PAGES; page++)
		OLED_WRITE(OLED_REG_DISPLAY_START, _oled_buffer[page], OLED_WIDTH);
	
	return OLED_OK;
}


oled_err_t oled_draw_battery_info(const uint16_t vbat_mv, 
								  const uint16_t vbat_pc)
{


	return OLED_OK;
}
