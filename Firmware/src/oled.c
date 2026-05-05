/******************************************************************************
* I2C OLED Text and "Graphics" Handler, Buffer and manager library - customised
* for TCRHMAG
* Really shitty design because I ran out of patience
*
* ADBeta (c)
******************************************************************************/
#include "oled.h"
#include "lib_i2c.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>


/*** Macro Functions *********************************************************/
#define OLED_WRITE(reg, buf, size)									 \
	do {															 \
		if(i2c_write_reg(_oled_dev, (reg), (buf), (size)) != I2C_OK) \
			return OLED_I2C_FAILED;									 \
	} while(0)



/*** Static Variables ********************************************************/
/// @brief Pointer to the OLED I2C Device for later use - use init() to set
static const i2c_device_t *_oled_dev = NULL;

// Framebuffer (page-based)
static uint8_t _oled_buffer[OLED_PAGES][OLED_WIDTH];


/*** Font Data ***************************************************************/
/// @brief 8x8 Limited Number/Symbol font for battery readout
static const uint8_t font8x8_data[][8] = {
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // ' '
	{0x46, 0x66, 0x30, 0x18, 0x0C, 0x66, 0x62, 0x00},   // '%'
	{0x00, 0x00, 0x60, 0x60, 0x00, 0x00, 0x00, 0x00},   // '.'
	{0x3E, 0x7F, 0x71, 0x59, 0x4D, 0x7F, 0x3E, 0x00},	// '0'
	{0x40, 0x42, 0x7F, 0x7F, 0x40, 0x40, 0x00, 0x00},	// '1'
	{0x62, 0x73, 0x59, 0x49, 0x6F, 0x66, 0x00, 0x00},	// '2'
	{0x22, 0x63, 0x49, 0x49, 0x7F, 0x36, 0x00, 0x00},	// '3'
	{0x18, 0x1C, 0x16, 0x53, 0x7F, 0x7F, 0x50, 0x00},	// '4'
	{0x27, 0x67, 0x45, 0x45, 0x7D, 0x39, 0x00, 0x00},	// '5'
	{0x3C, 0x7E, 0x4B, 0x49, 0x79, 0x30, 0x00, 0x00},	// '6'
	{0x03, 0x03, 0x71, 0x79, 0x0F, 0x07, 0x00, 0x00},	// '7'
	{0x36, 0x7F, 0x49, 0x49, 0x7F, 0x36, 0x00, 0x00},	// '8'
	{0x06, 0x4F, 0x49, 0x69, 0x3F, 0x1E, 0x00, 0x00},	// '9'
	{0x7C, 0x7E, 0x13, 0x13, 0x7E, 0x7C, 0x00, 0x00},	// 'A'
	{0x1F, 0x3F, 0x60, 0x60, 0x3F, 0x1F, 0x00, 0x00}	// 'V'
};

static const char font8x8_lut[] = {" %.0123456789AV"};



/*** Static Functions ********************************************************/
/// @brief Copies Data from the 8x8 font to the given data buffer pointer
/// @param str, string to append
/// @param ptr, Pointer to the data buffers position to append to
/// @return None
static void font8x8_to_framebuffer(const char *str, uint8_t *ptr)
{
	if(str == NULL || ptr == NULL) return;
	
	while(*str)
	{
		// Find the desired char in the small font LUT (incriment str too)
		const char *lut_ptr = strchr(font8x8_lut, *str++);
		if(lut_ptr == NULL) continue;
		
		// Copy the font data for the given char to the framebuffer
		memcpy(ptr, font8x8_data[lut_ptr - font8x8_lut], sizeof(font8x8_data[0]));
		ptr += sizeof(font8x8_data[0]);
	}
}


/// @brief Converts a given 
///
/// @return None
void append_formatted_2dp_string(const uint16_t val, char *str)
{
	uint16_t whole = val / 1000;          // volts
	uint16_t frac  = (val % 1000) / 10;   // hundredths of a volt

	str[0] = (whole / 10) + '0';
	str[1] = (whole % 10) + '0';
	str[2] = '.';
	str[3] = (frac / 10) + '0';
	str[4] = (frac % 10) + '0';
}





/*** Hardware Function Definitions *******************************************/
oled_err_t oled_init(const i2c_device_t *dev)
{
	if(dev == NULL) return OLED_INVALID_INPUT;

	// Set the static i2c device value to the given device
	_oled_dev = dev;

	const uint8_t init_arr[] = {
		0xAE,			// Display OFF
		0xD5, 0xF0,		// Clock
		0xA8, 0x3F,		// Multiplex (64)
		0xD3, 0x00,		// Offset
		0x40,			// Start line
		0x8D, 0x14,		// Charge pump
		0x20, 0x00,		// Horizontal addressing
		0xA1,			// Segment remap
		0xC8,			// COM scan dec
		0xDA, 0x12,		// COM pins
		0x81, 0x7F,		// Contrast
		0xD9, 0x22,		// Pre-charge
		0xDB, 0x40,		// VCOM detect
		0xA4,			// Resume RAM
		0xA6,			// Normal display
		0xAF			// Display ON
	};
	
	// Write the Initialisation Array to the Display
	OLED_WRITE(OLED_REG_COMMAND, init_arr,  sizeof(init_arr));
	
	return OLED_OK;
}


oled_err_t oled_set_cursor(const uint8_t x, const uint8_t y)
{
	if(x >= OLED_WIDTH || y >= OLED_PAGES) 
		return OLED_INVALID_INPUT;

	uint8_t pos_arr[3] = { (0xB0 | y), 
						   (0x00 | (x & 0x0F)), 
						   (0x10 | (x >> 4)) };
	// Set Page Address
	OLED_WRITE(OLED_REG_COMMAND, &pos_arr[0], 1);
	// Set lower column address
	OLED_WRITE(OLED_REG_COMMAND, &pos_arr[1], 1);
	// Set upper column address
	OLED_WRITE(OLED_REG_COMMAND, &pos_arr[2], 1);

	return OLED_OK;
}


oled_err_t oled_update(void)
{
	if(_oled_dev == NULL) return OLED_UNSET_DEVICE;
	
	// Write the OLED Display Data
	for(int page = 0; page < OLED_PAGES; page++)
	{	
		oled_set_cursor(0, page);
		OLED_WRITE(OLED_REG_DISPLAY_START, _oled_buffer[page], OLED_WIDTH);
	}

	return OLED_OK;
}




/*** Drawing API Functions ***************************************************/
oled_err_t oled_draw_battery_info(const uint16_t batt_mv,
								  const uint16_t batt_ma, 
								  const uint16_t vbat_pc)
{
	static char topbar_str[17] = "                ";

	

	append_formatted_2dp_string(batt_mv, &topbar_str[0]);



	// Draw the modified strings to the framebuffer
	font8x8_to_framebuffer(topbar_str, (uint8_t *)&_oled_buffer[0][0]);
	//font8x8_to_framebuffer(percent_str, (uint8_t *)&_oled_buffer[7][96]);
	return OLED_OK;
}
