/*
 * IncFile1.h
 *
 * Created: 14/03/2022 17:50:54
 *  Author: IMDL_USER_6
 */ 


#ifndef INCFILE1_H_
#define INCFILE1_H_



void QUAD_SPI_0_write(uint16_t address, uint32_t *data, uint8_t len);
void QUAD_SPI_0_read(uint16_t address,  uint32_t *dataIn, uint8_t len);
void pi_controller(float *out, float y_d, float y, float Kp, float Ki, float maxval);
void p_controller(float *out, float y_d, float y, float Kp, float maxval);
void unlocking_sequence_encoder(void);
void receive_data_from_encoder();
void set_encoder_offset_zero();
void SPI_1_write_block_and_wait(void *block, uint8_t size);


#endif /* INCFILE1_H_ */