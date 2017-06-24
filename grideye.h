/*
//Panasonic Grid-EYE Communication for Atmel Software Framework
//Jonathan Richardson
//
//Provides communication to the Panasonic Grid-EYE using the
//Atmel Software Framework. 
//
//Requires the use of ASF Two-wire Master Interface Driver.
*/

#include <clock.h>
#include "delay.h"
#include <asf.h>

#ifndef GRIDEYE_H_
#define GRIDEYE_H_

//static status_code_t init_test(void);


#define BUFFERC					5
#define GE_CALIBRATION_TIMES	BUFFERC
#define filterWidth				3
#define filterHeight			3
#define imageWidth				8
#define imageHeight				8
#define CONF_I2C_MASTER_MODULE    SERCOM1

typedef struct {
	int16_t data[64];
} GridEyeImage;


typedef struct {
	uint8_t address;
	struct i2c_master_module *twim_loc;
	uint8_t read_buffer[300], write_buffer[16];
	struct i2c_master_packet packet_RD, packet_WR;
	GridEyeImage image[BUFFERC];
	GridEyeImage difference;
	GridEyeImage bacground;
	GridEyeImage FilteredImag;
	int imagepointer;
	int initialframes;
	int16_t temperature;
	int8_t HiTempCount, LastHiTempCount, FastHiTempCount, XHitemp, XFirst, YHitemp, YFirst, Click, ClikSended, Delta[40];	
	uint8_t loop;
} GridEye;

uint8_t	ge_init						(GridEye *ge, struct i2c_master_module *twim, uint8_t grideye_address);
void	i2c_write_complete_callback	(struct i2c_master_module *const module);

/*Secondary Functions*/
uint8_t		ge_readData				(GridEye *ge);
uint8_t		ge_readDataAndSubstract	(GridEye *ge);
uint8_t		ge_readAverage			(GridEye *ge);

/*Direct Access*/
uint8_t		ge_writePacket			(GridEye *ge, uint8_t addr, uint8_t datalength, void* data);
uint8_t		ge_readPacket			(GridEye *ge, uint8_t addr, uint8_t datalength, void* data);
void		ge_setupPacket			(GridEye *ge, uint8_t addr, uint8_t datalength, void* data);
uint8_t		ge_setFPS				(GridEye *ge, bool fps_set_10);
uint8_t		ge_initialReset			(GridEye *ge);
uint8_t		ge_setInterrupt			(GridEye *ge, bool absolute, uint16_t upper, uint16_t lower, uint16_t hysteresis);
uint8_t		ge_readInterruptFlag	(GridEye *ge, bool clearFlag);
uint8_t		ge_clearInterruptFlag	(GridEye *ge);
uint8_t		ge_readFPS				(GridEye *ge);
uint8_t		ge_readTemp				(GridEye *ge);


/*GridEye Image Manipulation*/
void		geimage_copy			(GridEyeImage *from, GridEyeImage *to);
void		geimage_sum				(GridEyeImage *from, GridEyeImage *to);
void		geimage_subtract		(GridEyeImage *source, GridEyeImage *minus);
void		geimage_abs				(GridEyeImage *image);


#endif /* GRIDEYE_H_ */