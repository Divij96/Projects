/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */
#ifndef DRIVER_INIT_INCLUDED
#define DRIVER_INIT_INCLUDED

#include "atmel_start_pins.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <hal_atomic.h>
#include <hal_delay.h>
#include <hal_gpio.h>
#include <hal_init.h>
#include <hal_io.h>
#include <hal_sleep.h>

#include <hal_adc_sync.h>

#include <hal_adc_sync.h>

#include <hal_custom_logic.h>

#include <hal_evsys.h>

#include <hal_qspi_sync.h>

#include <spi_lite.h>
#include <tc_lite.h>
#include <tcc_lite.h>

extern struct adc_sync_descriptor CURR_A_TRQ_ADC;

extern struct adc_sync_descriptor CURR_B_ADC;

extern struct qspi_sync_descriptor ECAT_QSPI;

void CURR_A_TRQ_ADC_PORT_init(void);
void CURR_A_TRQ_ADC_CLOCK_init(void);
void CURR_A_TRQ_ADC_init(void);

void CURR_B_ADC_PORT_init(void);
void CURR_B_ADC_CLOCK_init(void);
void CURR_B_ADC_init(void);

void Hall_CCL_PORT_init(void);
void Hall_CCL_CLOCK_init(void);
void Hall_CCL_init(void);

void ECAT_QSPI_PORT_init(void);
void ECAT_QSPI_CLOCK_init(void);
void ECAT_QSPI_init(void);

void   ENC_SPI_PORT_init(void);
void   ENC_SPI_CLOCK_init(void);
int8_t ENC_SPI_init(void);

void HALL_Timer_CLOCK_init(void);

int8_t HALL_Timer_init(void);

void Moror_PWM_CLOCK_init(void);

void Moror_PWM_PORT_init(void);

int8_t Moror_PWM_init(void);

/**
 * \brief Perform system initialization, initialize pins and clocks for
 * peripherals
 */
void system_init(void);

#ifdef __cplusplus
}
#endif
#endif // DRIVER_INIT_INCLUDED
