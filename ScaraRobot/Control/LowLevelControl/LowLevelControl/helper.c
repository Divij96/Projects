/*
 * helper.c
 *
 * Created: 14/03/2022 17:50:40
 *  Author: IMDL_USER_6
 */ 
#include "driver_init.h"
#include "utils.h"
#include "helper.h"

//----for encoder
uint8_t encoder_raw_data[6] = {1, 2, 3, 4, 5, 6};
uint16_t encoder_multiturn_counter = 0;
float encoder_position = 0;
float encoder_position_in_rad = 0;
float encoder_offset = 0;
uint8_t encoder_Err = 0;
uint8_t encoder_Not_Warning = 0;
uint8_t encoder_crc_inv = 0;

float Fs = 25000; //Control loop frequency

void ECAT_QSPI_send(uint32_t* message, int buf_size, uint8_t instr,uint16_t address)
{

	struct _qspi_command cmd    = {
		.inst_frame.bits.width = 0,
		.inst_frame.bits.addr_en  = 1,
		.inst_frame.bits.data_en  = 1,
		.inst_frame.bits.tfr_type = QSPI_WRITE_ACCESS,
		.address =					address + (instr<<16),
		//.address_len
		.instruction              = instr,
		.buf_len                  = buf_size,
		.tx_buf                   = message,
	};

	qspi_sync_serial_run_command(&ECAT_QSPI, &cmd);
	//qspi_sync_deinit(&QUAD_SPI_0);
}

void ECAT_QSPI_read(uint32_t* message, int buf_size, uint8_t instr,uint16_t address)
{

	struct _qspi_command cmd    = {
		.inst_frame.bits.inst_en  = 0,
		.inst_frame.bits.data_en  = 1,
		.inst_frame.bits.tfr_type = QSPI_READ_ACCESS,
		.inst_frame.bits.addr_len=0,
		.inst_frame.bits.addr_en =1,
		.address                  =0x030000,
		.instruction              = 0x03,
		.buf_len                  = 64,
		.rx_buf                   = message,
	};

	qspi_sync_serial_run_command(&ECAT_QSPI, &cmd);
	//qspi_sync_deinit(&QUAD_SPI_0);
}

void pi_controller(float *out, float y_d, float y, float Kp, float Ki, float maxval)
{
	volatile float e = (y_d -y);
	out[0]= out[1]+ Kp*((e-out[2])+ (Ki*(e+out[2]))/Fs/2);
	
	if (out[0]>maxval) out[0] = maxval;
	else if (out[0]<-maxval) out[0] = -maxval;
	
	out[1]= out[0];
	out[2]= e;
}

void p_controller(float *out, float y_d, float y, float Kp, float maxval)
{
	volatile float e = (y_d -y);
	out[0]=  Kp*e;
	if (out[0]>maxval) out[0] = maxval;
	else if (out[0]<-maxval) out[0] = -maxval;
	
}


void SPI_1_write_block_and_wait(void *block, uint8_t size)
{

	uint8_t *b = (uint8_t *)block;
	while (size--) {
		hri_sercomspi_write_DATA_reg(SERCOM0, *b);
		while (!(hri_sercomspi_read_INTFLAG_reg(SERCOM0) & SERCOM_SPI_INTFLAG_TXC))
		;
		b++;
	}
}

void set_encoder_offset_zero()
{

	gpio_set_pin_level(ENC_CS, 0);
	ENC_SPI_read_block((void *)encoder_raw_data, 6); // output sensor data here
	encoder_multiturn_counter = (encoder_raw_data[0] << 8) | (encoder_raw_data[1] & 0xff);
	encoder_offset = (float)((encoder_raw_data[2] << 14) | (encoder_raw_data[3] << 6) | (encoder_raw_data[4] >> 2));
	encoder_Err = encoder_raw_data[4] & 0x02;
	encoder_Not_Warning = encoder_raw_data[4] & 0x01;
	gpio_set_pin_level(ENC_CS, 1);
}


void receive_data_from_encoder()
{
	//Encoder Counter in 21 Bit big
	gpio_set_pin_level(ENC_CS, 0);
	ENC_SPI_read_block((void *)encoder_raw_data, 6); // output sensor data here
	encoder_multiturn_counter = (encoder_raw_data[0] << 8) | (encoder_raw_data[1] & 0xff);
	encoder_position = (float)((encoder_raw_data[2] << 14) | (encoder_raw_data[3] << 6) | (encoder_raw_data[4] >> 2));
	encoder_position -= encoder_offset;
	encoder_position_in_rad = (encoder_position / 2097151) * 2 * 3.141592; // 2^21-1
	encoder_Err = encoder_raw_data[4] & 0x02;
	encoder_Not_Warning = encoder_raw_data[4] & 0x01;
	gpio_set_pin_level(ENC_CS, 1);
}


void unlocking_sequence_encoder()
{
	//not working
	uint8_t unlocking_sequence[9] = {0xcd, 0xef, 0x89, 0xab, 0x5a, 0x00, 0x00, 0x14, 0x18};
	for (uint8_t i = 0; i<9; i++)
	{
		uint8_t unlocking_sequence_i[6] = {unlocking_sequence[i], 0, 0, 0, 0, 0};
		gpio_set_pin_level(ENC_CS, 0);
		//SPI_1_read_block((void *)encoder_raw_data, 6); // output sensor data here
		SPI_1_write_block_and_wait((void *)unlocking_sequence_i, 6);
		delay_ms(1);
		gpio_set_pin_level(ENC_CS, 1);

	}
}
