/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */

#include "driver_examples.h"
#include "driver_init.h"
#include "utils.h"

/**
 * Example of using CURR_A_TRQ_ADC to generate waveform.
 */
void CURR_A_TRQ_ADC_example(void)
{
	uint8_t buffer[2];

	adc_sync_enable_channel(&CURR_A_TRQ_ADC, 0);

	while (1) {
		adc_sync_read_channel(&CURR_A_TRQ_ADC, 0, buffer, 2);
	}
}

/**
 * Example of using CURR_B_ADC to generate waveform.
 */
void CURR_B_ADC_example(void)
{
	uint8_t buffer[2];

	adc_sync_enable_channel(&CURR_B_ADC, 0);

	while (1) {
		adc_sync_read_channel(&CURR_B_ADC, 0, buffer, 2);
	}
}

/**
 * Example of using Hall_CCL.
 */
void Hall_CCL_example(void)
{
	custom_logic_enable();
	/* Customer logic now works. */
}

/**
 * Example of using ECAT_QSPI to get N25Q256A status value,
 * and check bit 0 which indicate embedded operation is busy or not.
 */
void ECAT_QSPI_example(void)
{
	uint8_t              status = 0xFF;
	struct _qspi_command cmd    = {
        .inst_frame.bits.inst_en  = 1,
        .inst_frame.bits.data_en  = 1,
        .inst_frame.bits.tfr_type = QSPI_READ_ACCESS,
        .instruction              = 0x05,
        .buf_len                  = 1,
        .rx_buf                   = &status,
    };

	qspi_sync_enable(&ECAT_QSPI);
	while (status & (1 << 0)) {
		qspi_sync_serial_run_command(&ECAT_QSPI, &cmd);
	}
	qspi_sync_deinit(&ECAT_QSPI);
}

/**
 * Example of using ENC_SPI to write "Hello World" using the IO abstraction.
 */
static uint8_t example_ENC_SPI[12] = "Hello World!";

void ENC_SPI_example(void)
{
	ENC_SPI_write_block((void *)example_ENC_SPI, 12);
}
