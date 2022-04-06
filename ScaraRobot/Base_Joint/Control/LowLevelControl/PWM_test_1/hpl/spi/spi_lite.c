
/**
 * \file
 *
 * \brief SPI related functionality implementation.
 *
 * Copyright (c) 2017 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */

#include "spi_lite.h"
#include <utils_assert.h>

/**
 * \brief Initialize SPI interface
 */
int8_t ENC_SPI_init()
{

	if (!hri_sercomspi_is_syncing(SERCOM0, SERCOM_SPI_SYNCBUSY_SWRST)) {
		uint32_t mode = SERCOM_SPI_CTRLA_MODE(3);
		if (hri_sercomspi_get_CTRLA_reg(SERCOM0, SERCOM_SPI_CTRLA_ENABLE)) {
			hri_sercomspi_clear_CTRLA_ENABLE_bit(SERCOM0);
			hri_sercomspi_wait_for_sync(SERCOM0, SERCOM_SPI_SYNCBUSY_ENABLE);
		}
		hri_sercomspi_write_CTRLA_reg(SERCOM0, SERCOM_SPI_CTRLA_SWRST | mode);
	}
	hri_sercomspi_wait_for_sync(SERCOM0, SERCOM_SPI_SYNCBUSY_SWRST);

	hri_sercomspi_write_CTRLA_reg(
	    SERCOM0,
	    0 << SERCOM_SPI_CTRLA_DORD_Pos           /* Data Order: disabled */
	        | 0 << SERCOM_SPI_CTRLA_CPOL_Pos     /* Clock Polarity: disabled */
	        | 0 << SERCOM_SPI_CTRLA_CPHA_Pos     /* Clock Phase: disabled */
	        | 0 << SERCOM_SPI_CTRLA_FORM_Pos     /* Frame Format: 0 */
	        | 0 << SERCOM_SPI_CTRLA_IBON_Pos     /* Immediate Buffer Overflow Notification: disabled */
	        | 1 << SERCOM_SPI_CTRLA_RUNSTDBY_Pos /* Run In Standby: enabled */
	        | 3 << SERCOM_SPI_CTRLA_MODE_Pos);   /* Operating Mode: 3 */

	hri_sercomspi_write_CTRLA_DOPO_bf(SERCOM0, SERCOM0_TXPO);
	hri_sercomspi_write_CTRLA_DIPO_bf(SERCOM0, SERCOM0_RXPO);

	hri_sercomspi_write_CTRLB_reg(SERCOM0,
	                              1 << SERCOM_SPI_CTRLB_RXEN_Pos          /* Receiver Enable: enabled */
	                                  | 1 << SERCOM_SPI_CTRLB_MSSEN_Pos   /* Master Slave Select Enabl: enabled */
	                                  | 0 << SERCOM_SPI_CTRLB_AMODE_Pos   /* Address Mode: 0 */
	                                  | 0 << SERCOM_SPI_CTRLB_SSDE_Pos    /* Slave Select Low Detect Enable: disabled */
	                                  | 0 << SERCOM_SPI_CTRLB_PLOADEN_Pos /* Slave Data Preload Enable: disabled */
	                                  | 0);                               /* Character Size: 0 */

	hri_sercomspi_write_BAUD_reg(SERCOM0, SERCOM0_BAUD_RATE);

	// hri_sercomspi_write_DBGCTRL_reg(SERCOM0,0 << SERCOM_SPI_DBGCTRL_DBGSTOP_Pos); /* Debug Stop Mode: disabled */

	// hri_sercomspi_write_INTEN_reg(SERCOM0,0 << SERCOM_SPI_INTENSET_ERROR_Pos /* Error Interrupt Enable: disabled */
	//		 | 0 << SERCOM_SPI_INTENSET_SSL_Pos /* Slave Select Low Interrupt Enable: disabled */
	//		 | 0 << SERCOM_SPI_INTENSET_RXC_Pos /* Receive Complete Interrupt Enable: disabled */
	//		 | 0 << SERCOM_SPI_INTENSET_TXC_Pos /* Transmit Complete Interrupt Enable: disabled */
	//		 | 0 << SERCOM_SPI_INTENSET_DRE_Pos); /* Data Register Empty Interrupt Enable: disabled */

	hri_sercomspi_write_CTRLA_ENABLE_bit(SERCOM0, 1 << SERCOM_SPI_CTRLA_ENABLE_Pos); /* Enable: enabled */

	return 0;
}

/**
 * \brief Enable SPI module
 */
void ENC_SPI_enable()
{
	hri_sercomspi_set_CTRLA_ENABLE_bit(SERCOM0);
}

/**
 * \brief Disable SPI module
 */
void ENC_SPI_disable()
{
	hri_sercomspi_clear_CTRLA_ENABLE_bit(SERCOM0);
}

/**
 * \brief Exchange_byte in SPI module
 */
uint32_t ENC_SPI_exchange_data(uint32_t data)
{
	/* If settings are not applied (pending), we can not go on */
	if (hri_sercomspi_is_syncing(
	        SERCOM0, (SERCOM_SPI_SYNCBUSY_SWRST | SERCOM_SPI_SYNCBUSY_ENABLE | SERCOM_SPI_SYNCBUSY_CTRLB))) {
		return ERR_BUSY;
	}

	hri_sercomspi_write_DATA_reg(SERCOM0, data);
	while (!(hri_sercomspi_read_INTFLAG_reg(SERCOM0) & SERCOM_SPI_INTFLAG_RXC))
		;
	return hri_sercomspi_read_DATA_reg(SERCOM0);
}

void ENC_SPI_exchange_block(void *block, uint8_t size)
{

	uint8_t *b = (uint8_t *)block;

	while (size--) {
		hri_sercomspi_write_DATA_reg(SERCOM0, *b);
		while (!(hri_sercomspi_read_INTFLAG_reg(SERCOM0) & SERCOM_SPI_INTFLAG_RXC))
			;
		*b = hri_sercomspi_read_DATA_reg(SERCOM0);
		b++;
	}
}

void ENC_SPI_write_block(void *block, uint8_t size)
{

	uint8_t *b = (uint8_t *)block;
	while (size--) {
		hri_sercomspi_write_DATA_reg(SERCOM0, *b);
		while (!(hri_sercomspi_read_INTFLAG_reg(SERCOM0) & SERCOM_SPI_INTFLAG_RXC))
			;
		b++;
	}
}

void ENC_SPI_read_block(void *block, uint8_t size)
{

	uint8_t *b = (uint8_t *)block;
	while (size--) {
		hri_sercomspi_write_DATA_reg(SERCOM0, 0);
		while (!(hri_sercomspi_read_INTFLAG_reg(SERCOM0) & SERCOM_SPI_INTFLAG_RXC))
			;
		*b = hri_sercomspi_read_DATA_reg(SERCOM0);
		b++;
	}
}
