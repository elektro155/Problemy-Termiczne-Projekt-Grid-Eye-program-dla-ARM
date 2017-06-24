/*
//Panasonic Grid-EYE Communication for Atmel Software Framework
//Jonathan Richardson
//
//Provides communication to the Panasonic Grid-EYE using the
//Atmel Software Framework. 
//
//Requires the use of ASF Two-wire Master Interface Driver.
*/
#include "grideye.h"

uint8_t ge_init (GridEye *ge, struct i2c_master_module *twim, uint8_t address_number)
{
	//set the I2C address of the Grid Eye instance

	//TWIM port to use
	ge->twim_loc = twim;
	
	//Set default structure variables
	ge->imagepointer = 0;
	ge->temperature = 0;

	for(int i = 0; i < BUFFERC; i++)
		for(int j = 0; j < 64; j++)
			ge->image[i].data[j] = 0;
			
	for(int j = 0; j < 64; j++)
	{
		ge->difference.data[j] = 0;
	}
	
	//Setup the TWIM module with the proper settings for the Grid Eye
	struct i2c_master_config config_i2c_master;
	i2c_master_get_config_defaults(&config_i2c_master);

	config_i2c_master.buffer_timeout = 65535;
	config_i2c_master.baud_rate		 = 400;
	config_i2c_master.pinmux_pad0    = PINMUX_PA16C_SERCOM1_PAD0;
	config_i2c_master.pinmux_pad1    = PINMUX_PA17C_SERCOM1_PAD1;	
	
	
	while(i2c_master_init(ge->twim_loc, CONF_I2C_MASTER_MODULE, &config_i2c_master) != STATUS_OK);
	delay_ms(1);
	i2c_master_enable(ge->twim_loc);
	delay_ms(1);
	ge->packet_RD.address		  = address_number == 0 ? 0b1101000 : 0b1101001;
	ge->packet_RD.data_length	  = 128;
	ge->packet_RD.data			  = ge->read_buffer;
	ge->packet_RD.ten_bit_address = false;
	ge->packet_RD.high_speed	  = false;
		
	ge->packet_WR.address		  = address_number == 0 ? 0b1101000 : 0b1101001;
	ge->packet_WR.data_length	  = 16;
	ge->packet_WR.data			  = ge->write_buffer;
	ge->packet_WR.ten_bit_address = false;
	ge->packet_WR.high_speed	  = false;
	//setup callback
	delay_ms(1);
	i2c_master_register_callback(ge->twim_loc, i2c_write_complete_callback, I2C_MASTER_CALLBACK_WRITE_COMPLETE);

	i2c_master_enable_callback(ge->twim_loc, I2C_MASTER_CALLBACK_WRITE_COMPLETE);
	delay_ms(2);

	return ge_initialReset(ge);
}

void i2c_write_complete_callback(struct i2c_master_module *const module)
{

}

uint8_t ge_readData (GridEye *ge)
{
	ge->packet_WR.data_length = 1;
	ge->packet_WR.data[0] = 0x80;	
	if(i2c_master_write_packet_wait_no_stop(ge->twim_loc, &ge->packet_WR) != STATUS_OK)
	{
		return 0;
	}	
	//Read full 128 bytes of data from Grid Eye
	ge->packet_RD.data_length = 128;
	if(i2c_master_read_packet_wait(ge->twim_loc, &ge->packet_RD)  != STATUS_OK)
	{
		return 0;
	}		
	//Convert read buffer to actual image
	for(int i = 0; i < 64; i++)
		ge->image[0].data[i] = (ge->read_buffer[2*i] | ((ge->read_buffer[2*i+1] & 0x7) << 8));

	return 1;
}

uint8_t	ge_readDataAndSubstract	(GridEye *ge)
{
	ge->packet_WR.data_length = 1;
	ge->packet_WR.data[0] = 0x80;
	if(i2c_master_write_packet_wait_no_stop(ge->twim_loc, &ge->packet_WR) != STATUS_OK)
	{
		return 0;
	}
	//Read full 128 bytes of data from Grid Eye
	ge->packet_RD.data_length = 128;
	if(i2c_master_read_packet_wait(ge->twim_loc, &ge->packet_RD)  != STATUS_OK)
	{
		return 0;
	}
	//Convert read buffer to actual image
	for(int i = 0; i < 64; i++)
	{	
		ge->FilteredImag.data[i] = 10 + ((ge->FilteredImag.data[i] * 2) + (ge->read_buffer[2*i] | ((ge->read_buffer[2*i+1] & 0x7) << 8))) / 3;
		if(ge->FilteredImag.data[i] < ge->bacground.data[i])
		{
			ge->difference.data[i] = ge->bacground.data[i] - ge->FilteredImag.data[i];
		}
		else
		{
			ge->difference.data[i] = ge->FilteredImag.data[i] - ge->bacground.data[i];
		}		
	}

	return 1;
}
uint8_t ge_readAverage(GridEye *ge)
{
	//reset the difference array
	for(int i = 0; i < 64; i++)
		ge->bacground.data[i] = 0;

	//fill the read buffer while storing the sum of the buffer in difference
	for(int i = 0; i < BUFFERC; i++)
	{
		if(!ge_readData(ge))
			return false;
		delay_ms(105);
		geimage_sum(ge->image[ge->imagepointer].data, ge->bacground.data);
	}

	//finish taking take the average of the difference array
	for(int i = 0; i < 64; i++)
		ge->difference.data[i] = ge->bacground.data[i] /= BUFFERC;

	return true;
}

void ge_setupPacket(GridEye *ge, uint8_t addr, uint8_t datalength, void* data)
{
	//create the structure of a new packet
		ge->packet_RD.address = ge->address;
		ge->packet_RD.high_speed = 0;
		ge->packet_RD.hs_master_code = 0;
		ge->packet_RD.data = data;
		ge->packet_RD.data_length = datalength;
		ge->packet_RD.ten_bit_address = false;	
}

uint8_t ge_writePacket(GridEye *ge, uint8_t addr, uint8_t datalength, void* data)
{
	//setup write packet and send
	ge_setupPacket(ge, addr, datalength, data);
	return i2c_master_write_packet_job(ge->twim_loc, &ge->packet_RD);
}

uint8_t ge_readPacket(GridEye *ge, uint8_t addr, uint8_t datalength, void* data)
{
	//setup request packet and send
	ge_setupPacket(ge, addr, datalength, data);
	return i2c_master_read_packet_job(ge->twim_loc, &ge->packet_RD);
}

uint8_t ge_setFPS (GridEye *ge, bool fps_set_10)
{
	//setup packet and write to Grid Eye
	ge->packet_WR.data[0] = 0x02;
	ge->packet_WR.data[1] = !fps_set_10;
	ge->packet_WR.data_length = 2;
	if(i2c_master_write_packet_wait(ge->twim_loc, &ge->packet_WR) != STATUS_OK)
	{
		return 0;
	}

	return 1;
}

uint8_t ge_readFPS (GridEye *ge)
{
	//Read Grid Eye setting
	uint8_t databuffer = 0;
	if(!ge_readPacket(ge, 0x02, 1, &databuffer))
		return 0xFF;
	return (databuffer &= 1==0?10:1);
}

uint8_t ge_readTemp (GridEye *ge)
{
	int16_t rawdata;

	ge->packet_WR.data[0] = 0x0E;
	ge->packet_WR.data_length = 1;
	if(i2c_master_write_packet_wait_no_stop(ge->twim_loc, &ge->packet_WR) != STATUS_OK)
	{	
		return 0;
	}	
	ge->packet_RD.data_length = 2;
	if(i2c_master_read_packet_wait(ge->twim_loc, &ge->packet_RD)  != STATUS_OK)
	{		
		return 0;
	}	
	rawdata = ge->packet_RD.data[0] | (0x0F & ge->packet_RD.data[1]) << 4;
	//handle negative numbers, the sign bit is stored in bit 11
	//when read directly from the Grid Eye
	if((rawdata >> 11) & 1)
		rawdata |= 1 << 15;

	//mask out all the bits that are not needed
	rawdata &= 0x87FF;

	ge->temperature = rawdata * 0.25;
	
	return true;
}

uint8_t ge_initialReset (GridEye *ge)
{	
	ge->packet_WR.data[0] = 0x01;
	ge->packet_WR.data[1] = 0x3F;
	ge->packet_WR.data_length = 2;
	if(i2c_master_write_packet_wait(ge->twim_loc, &ge->packet_WR) != STATUS_OK)
	{
		return 0;
	}
	return 1;
}

uint8_t ge_setInterrupt (GridEye *ge, bool absolute, uint16_t upper, uint16_t lower, uint16_t hysteresis)
{
	//mask variables to 12-bit and create the full packet to send		
	ge->packet_WR.data[0] = 0x08;
	ge->packet_WR.data[1] = (upper & 0xF00)>> 4;
	ge->packet_WR.data[2] = (upper & 0xFF);
	ge->packet_WR.data[3] = (lower & 0xF00)>> 4;
	ge->packet_WR.data[4] = (lower & 0xFF);
	ge->packet_WR.data[5] = (hysteresis & 0xF00)>> 4;
	ge->packet_WR.data[6] = (hysteresis & 0xFF);
	ge->packet_WR.data_length = 7;
	if(i2c_master_write_packet_wait(ge->twim_loc, &ge->packet_WR) != STATUS_OK)
	{
		return 0;
	}

	//send packet to Grid Eye
	ge->packet_WR.data[0] = 0x03;
	ge->packet_WR.data[1] = 0x01 | (absolute?0x02:0x00);
	ge->packet_WR.data_length = 2;
	if(i2c_master_write_packet_wait(ge->twim_loc, &ge->packet_WR) != STATUS_OK)
	{
		return 0;
	}

	return 1;
}

uint8_t ge_readInterruptFlag (GridEye *ge, bool clearFlag)
{
	uint8_t databuffer;
	bool status = false;

	//read interrupt flag from register
	if(!ge_readPacket(ge, 0x04, 1, &databuffer))
		return false;

	//buffer to boolean
	status = (databuffer >> 1) & 1;
	
	//if chosen, clear the interrupt flag
	if(clearFlag && status)
		ge_clearInterruptFlag(ge);

	return status;
}

uint8_t ge_clearInterruptFlag (GridEye *ge)
{
	//create and send clear packet to flag register
	uint8_t clearPacket = 0x02;
	return ge_writePacket(ge, 0x05, 1, &clearPacket);
}

void geimage_copy(GridEyeImage *from, GridEyeImage *to)
{
	for(int i = 0; i < 64; i++)
		to->data[i] = from->data[i];
}

void geimage_sum(GridEyeImage *from, GridEyeImage *to)
{
	for(int i = 0; i < 64; i++)
		to->data[i] += from->data[i];
}

void geimage_subtract(GridEyeImage *source, GridEyeImage *minus)
{
	for(int i = 0; i < 64; i++)
		source->data[i] -= minus->data[i];
}

void geimage_abs(GridEyeImage *image)
{
	for(int i = 0; i < 64; i++)
		image->data[i] = abs(image->data[i]);
}

