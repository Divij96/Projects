/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */

#include "driver_init.h"
#include <peripheral_clk_config.h>
#include <utils.h>
#include <hal_init.h>

#include <hpl_adc_base.h>
#include <hpl_adc_base.h>

struct adc_sync_descriptor CURR_A_TRQ_ADC;

struct adc_sync_descriptor CURR_B_ADC;

struct qspi_sync_descriptor ECAT_QSPI;

void CURR_A_TRQ_ADC_PORT_init(void)
{

	// Disable digital pin circuitry
	gpio_set_pin_direction(PA02, GPIO_DIRECTION_OFF);

	gpio_set_pin_function(PA02, PINMUX_PA02B_ADC0_AIN0);

	// Disable digital pin circuitry
	gpio_set_pin_direction(PA03, GPIO_DIRECTION_OFF);

	gpio_set_pin_function(PA03, PINMUX_PA03B_ADC0_AIN1);
}

void CURR_A_TRQ_ADC_CLOCK_init(void)
{
	hri_mclk_set_APBDMASK_ADC0_bit(MCLK);
	hri_gclk_write_PCHCTRL_reg(GCLK, ADC0_GCLK_ID, CONF_GCLK_ADC0_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
}

void CURR_A_TRQ_ADC_init(void)
{
	CURR_A_TRQ_ADC_CLOCK_init();
	CURR_A_TRQ_ADC_PORT_init();
	adc_sync_init(&CURR_A_TRQ_ADC, ADC0, (void *)NULL);
}

void CURR_B_ADC_PORT_init(void)
{

	// Disable digital pin circuitry
	gpio_set_pin_direction(PB08, GPIO_DIRECTION_OFF);

	gpio_set_pin_function(PB08, PINMUX_PB08B_ADC1_AIN0);
}

void CURR_B_ADC_CLOCK_init(void)
{
	hri_mclk_set_APBDMASK_ADC1_bit(MCLK);
	hri_gclk_write_PCHCTRL_reg(GCLK, ADC1_GCLK_ID, CONF_GCLK_ADC1_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
}

void CURR_B_ADC_init(void)
{
	CURR_B_ADC_CLOCK_init();
	CURR_B_ADC_PORT_init();
	adc_sync_init(&CURR_B_ADC, ADC1, (void *)NULL);
}

void Hall_CCL_PORT_init(void)
{
}

void Hall_CCL_CLOCK_init(void)
{
	hri_mclk_set_APBCMASK_CCL_bit(MCLK);
	hri_gclk_write_PCHCTRL_reg(GCLK, CCL_GCLK_ID, CONF_GCLK_CCL_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
}

void Hall_CCL_init(void)
{
	Hall_CCL_CLOCK_init();
	custom_logic_init();
	Hall_CCL_PORT_init();
}

void EVESYS_init(void)
{
	hri_gclk_write_PCHCTRL_reg(GCLK, EVSYS_GCLK_ID_0, CONF_GCLK_EVSYS_CHANNEL_0_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_gclk_write_PCHCTRL_reg(GCLK, EVSYS_GCLK_ID_1, CONF_GCLK_EVSYS_CHANNEL_1_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));

	hri_mclk_set_APBBMASK_EVSYS_bit(MCLK);

	event_system_init();
}

void ECAT_QSPI_PORT_init(void)
{

	// Set pin direction to input
	gpio_set_pin_direction(PB11, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(PB11,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PB11, PINMUX_PB11H_QSPI_CS);

	gpio_set_pin_direction(PA08,
	                       // <y> Pin direction
	                       // <id> pad_direction
	                       // <GPIO_DIRECTION_OFF"> Off
	                       // <GPIO_DIRECTION_IN"> In
	                       // <GPIO_DIRECTION_OUT"> Out
	                       GPIO_DIRECTION_OUT);

	gpio_set_pin_level(PA08,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	gpio_set_pin_pull_mode(PA08,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PA08,
	                      // <y> Pin function
	                      // <id> pad_function
	                      // <i> Auto : use driver pinmux if signal is imported by driver, else turn off function
	                      // <PINMUX_PA08H_QSPI_DATA0"> Auto
	                      // <GPIO_PIN_FUNCTION_OFF"> Off
	                      // <GPIO_PIN_FUNCTION_A"> A
	                      // <GPIO_PIN_FUNCTION_B"> B
	                      // <GPIO_PIN_FUNCTION_C"> C
	                      // <GPIO_PIN_FUNCTION_D"> D
	                      // <GPIO_PIN_FUNCTION_E"> E
	                      // <GPIO_PIN_FUNCTION_F"> F
	                      // <GPIO_PIN_FUNCTION_G"> G
	                      // <GPIO_PIN_FUNCTION_H"> H
	                      // <GPIO_PIN_FUNCTION_I"> I
	                      // <GPIO_PIN_FUNCTION_J"> J
	                      // <GPIO_PIN_FUNCTION_K"> K
	                      // <GPIO_PIN_FUNCTION_L"> L
	                      // <GPIO_PIN_FUNCTION_M"> M
	                      // <GPIO_PIN_FUNCTION_N"> N
	                      PINMUX_PA08H_QSPI_DATA0);

	gpio_set_pin_direction(PA09,
	                       // <y> Pin direction
	                       // <id> pad_direction
	                       // <GPIO_DIRECTION_OFF"> Off
	                       // <GPIO_DIRECTION_IN"> In
	                       // <GPIO_DIRECTION_OUT"> Out
	                       GPIO_DIRECTION_OUT);

	gpio_set_pin_level(PA09,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	gpio_set_pin_pull_mode(PA09,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PA09,
	                      // <y> Pin function
	                      // <id> pad_function
	                      // <i> Auto : use driver pinmux if signal is imported by driver, else turn off function
	                      // <PINMUX_PA09H_QSPI_DATA1"> Auto
	                      // <GPIO_PIN_FUNCTION_OFF"> Off
	                      // <GPIO_PIN_FUNCTION_A"> A
	                      // <GPIO_PIN_FUNCTION_B"> B
	                      // <GPIO_PIN_FUNCTION_C"> C
	                      // <GPIO_PIN_FUNCTION_D"> D
	                      // <GPIO_PIN_FUNCTION_E"> E
	                      // <GPIO_PIN_FUNCTION_F"> F
	                      // <GPIO_PIN_FUNCTION_G"> G
	                      // <GPIO_PIN_FUNCTION_H"> H
	                      // <GPIO_PIN_FUNCTION_I"> I
	                      // <GPIO_PIN_FUNCTION_J"> J
	                      // <GPIO_PIN_FUNCTION_K"> K
	                      // <GPIO_PIN_FUNCTION_L"> L
	                      // <GPIO_PIN_FUNCTION_M"> M
	                      // <GPIO_PIN_FUNCTION_N"> N
	                      PINMUX_PA09H_QSPI_DATA1);

	// Set pin direction to input
	gpio_set_pin_direction(PB10, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(PB10,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PB10, PINMUX_PB10H_QSPI_SCK);
}

void ECAT_QSPI_CLOCK_init(void)
{
	hri_mclk_set_AHBMASK_QSPI_bit(MCLK);
	hri_mclk_set_AHBMASK_QSPI_2X_bit(MCLK);
	hri_mclk_set_APBCMASK_QSPI_bit(MCLK);
}

void ECAT_QSPI_init(void)
{
	ECAT_QSPI_CLOCK_init();
	qspi_sync_init(&ECAT_QSPI, QSPI);
	ECAT_QSPI_PORT_init();
}

void ENC_SPI_PORT_init(void)
{

	gpio_set_pin_level(PA04,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(PA04, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(PA04, PINMUX_PA04D_SERCOM0_PAD0);

	gpio_set_pin_level(PA05,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(PA05, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(PA05, PINMUX_PA05D_SERCOM0_PAD1);

	// Set pin direction to input
	gpio_set_pin_direction(PA06, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(PA06,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PA06, PINMUX_PA06D_SERCOM0_PAD2);
}

void ENC_SPI_CLOCK_init(void)
{
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM0_GCLK_ID_CORE, CONF_GCLK_SERCOM0_CORE_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM0_GCLK_ID_SLOW, CONF_GCLK_SERCOM0_SLOW_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));

	hri_mclk_set_APBAMASK_SERCOM0_bit(MCLK);
}

void HALL_Timer_CLOCK_init(void)
{
	hri_mclk_set_APBAMASK_TC0_bit(MCLK);

	hri_mclk_set_APBAMASK_TC1_bit(MCLK);
	hri_gclk_write_PCHCTRL_reg(GCLK, TC0_GCLK_ID, CONF_GCLK_TC0_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
}

void Moror_PWM_PORT_init(void)
{

	gpio_set_pin_function(PA21, PINMUX_PA21G_TCC0_WO1);

	gpio_set_pin_function(PA22, PINMUX_PA22G_TCC0_WO2);

	gpio_set_pin_function(PA16, PINMUX_PA16G_TCC0_WO4);

	gpio_set_pin_function(PA17, PINMUX_PA17G_TCC0_WO5);

	gpio_set_pin_function(PA18, PINMUX_PA18G_TCC0_WO6);

	gpio_set_pin_function(PA19, PINMUX_PA19G_TCC0_WO7);
}

void Moror_PWM_CLOCK_init(void)
{
	hri_mclk_set_APBBMASK_TCC0_bit(MCLK);
	hri_gclk_write_PCHCTRL_reg(GCLK, TCC0_GCLK_ID, CONF_GCLK_TCC0_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
}

void system_init(void)
{
	init_mcu();

	// GPIO on PA14

	gpio_set_pin_level(ENC_CS,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(ENC_CS, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(ENC_CS, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA23

	// Set pin direction to input
	gpio_set_pin_direction(Hall_B, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(Hall_B,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_UP);

	gpio_set_pin_function(Hall_B, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA24

	// Set pin direction to input
	gpio_set_pin_direction(Hall_A, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(Hall_A,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_UP);

	gpio_set_pin_function(Hall_A, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB22

	// Set pin direction to input
	gpio_set_pin_direction(Hall_C, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(Hall_C,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_UP);

	gpio_set_pin_function(Hall_C, GPIO_PIN_FUNCTION_OFF);

	CURR_A_TRQ_ADC_init();

	CURR_B_ADC_init();

	Hall_CCL_init();

	EVESYS_init();

	ECAT_QSPI_init();

	ENC_SPI_CLOCK_init();
	ENC_SPI_init();
	ENC_SPI_PORT_init();

	HALL_Timer_CLOCK_init();

	HALL_Timer_init();

	Moror_PWM_CLOCK_init();

	Moror_PWM_PORT_init();

	Moror_PWM_init();
}
