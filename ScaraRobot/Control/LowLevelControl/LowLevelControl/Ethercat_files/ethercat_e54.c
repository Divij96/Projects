/*
 * ethercat_e54.c
 *
 * Created: 02/03/2021 10:49:41
 *  Author: Nick-XMG
 */ 

// ----------------------------------------------------------------------
// Header Files
// ----------------------------------------------------------------------
#include "atmel_start.h"
#include "driver_init.h"

#include "ethercat_e54.h"


void TC7_Handler(void){
	TC7->COUNT16.INTFLAG.bit.OVF =1;
		tx_ethercat = true;
	if(tx_ethercat_done){
		volatile int i=0;
		
		gpio_set_pin_level(ECAT_SPI_CS_PIN, false);  // SPI_Slave Select LOW
		DMAC->Channel[0].CHCTRLB.reg = 0x2; // Resume				DMAC->Channel[1].CHCTRLB.reg = 0x2; // Resume		tx_ethercat_done = false;	}	else {				gpio_set_pin_level(ECAT_SPI_CS_PIN, true);			// SPI_Slave Select HIGH
		tx_ethercat_done = false;		gpio_set_pin_level(ECAT_SPI_CS_PIN, false); // SPI_Slave Select LOW		_dma_enable_transaction(0,false);		_dma_enable_transaction(1,false);		tx_ethercat_done = false;	}

	};

extern void One_ms_cycle_callback(void)
{


}

// ----------------------------------------------------------------------
// Callbacks
// ----------------------------------------------------------------------

static void transfer_error(struct _dma_resource *resource)
{
	uint8_t error = 1;
	//	spi_select_slave(&spi_master_instance, &slave, false);
	//DMAC->CHID.reg = resource->channel_id;
	//DMAC->CHCTRLB.bit.CMD = 0x02;
	//DMAC->CHINTFLAG.bit.SUSP = 1;
	//responde_spi_master = true;
	//com_state = com_error;
}

static void LAN9252_rx_done(struct _dma_resource *resource)
{
	gpio_set_pin_level(ECAT_SPI_CS_PIN, true);
	gpio_set_pin_level(ECAT_SPI_CS_PIN, false);
	DMAC->Channel[0].CHCTRLB.reg = 0x2; // Resume	DMAC->Channel[1].CHCTRLB.reg = 0x2; // Resume	tx_ethercat_done = false;
}


static void LAN9252_rx_susp(struct _dma_resource *resource)
{
	gpio_set_pin_level(ECAT_SPI_CS_PIN, true);		//volatile uint32_t *pointer = ((DMAC->WRBADDR.reg)+0x10*resource->channel_id+12);	volatile uint32_t *pointer = ((DMAC->WRBADDR.reg)+12);	if (*pointer != &spi_rx_write_fifo_dma_descriptor){		DMAC->Channel[0].CHCTRLB.reg = 0x2; // Resume		gpio_set_pin_level(ECAT_SPI_CS_PIN, false);		DMAC->Channel[1].CHCTRLB.reg = 0x2; // Resume		tx_ethercat_done = false;	}	else{		tx_ethercat_done = true;		NVIC_EnableIRQ(TC7_IRQn);	}
}



// ----------------------------------------------------------------------
// 	functions
// ----------------------------------------------------------------------

void config_ethercat_tc(void)
{
	//tc_syncro_task.interval = 100;
	//tc_syncro_task.cb		= One_ms_cycle_callback;
	//tc_syncro_task.mode     = TIMER_TASK_REPEAT;
	//timer_add_task(&TC_ECAT, &tc_syncro_task);		
	//timer_start(&TC_ECAT);	
}


void configure_ethercat_dma_descriptors(void)
{
	//TC_ECAT_init();
	
	// ----------------------------------------------------------------------
	// DMAC Descriptors
	// ----------------------------------------------------------------------
	//abort actual_fifo
	setup_transfer_descriptor(DMA_TRANSFER_DESCRIPTOR_TX, &spi_abort_fifo_dma_descriptor, abort_fifo_start, abort_fifo_length, &spi_clear_rd_fifo_dma_descriptor, DMAC_BTCTRL_BLOCKACT_SUSPEND_Val);			//DMA_BLOCK_ACTION_SUSPEND
	//clear initial read data from LAN9252 to avoid errors.
	setup_transfer_descriptor(DMA_TRANSFER_DESCRIPTOR_TX,&spi_clear_rd_fifo_dma_descriptor,wr_pdram_start,wr_pdram_lenght+4*write_var_num,&spi_write_cl_fifo_dma_descriptor,DMAC_BTCTRL_BLOCKACT_SUSPEND_Val);	//DMA_BLOCK_ACTION_SUSPEND
	setup_transfer_descriptor(DMA_TRANSFER_DESCRIPTOR_TX,&spi_write_cl_fifo_dma_descriptor,cl_pdram_start,cl_pdram_lenght,&spi_write_fifo_dma_descriptor,DMAC_BTCTRL_BLOCKACT_SUSPEND_Val);						//DMA_BLOCK_ACTION_SUSPEND
	// write fifo registers
	setup_transfer_descriptor(DMA_TRANSFER_DESCRIPTOR_TX,&spi_write_fifo_dma_descriptor,wr_pdram_start,wr_pdram_lenght,&spi_write2ram_dma_descriptor,DMAC_BTCTRL_BLOCKACT_NOACT_Val);							//DMA_BLOCK_ACTION_NOACT
	setup_transfer_descriptor(DMA_TRANSFER_DESCRIPTOR_TX,&spi_write2ram_dma_descriptor,&ram_buffer[ram_real_wr_start],4*write_var_num,&spi_config_fifo_dma_descriptor,DMAC_BTCTRL_BLOCKACT_SUSPEND_Val);		//DMA_BLOCK_ACTION_SUSPEND
	// write fifo configuration
	setup_transfer_descriptor(DMA_TRANSFER_DESCRIPTOR_TX,&spi_config_fifo_dma_descriptor,cf_pdram_start,cf_pdram_lenght,&spi_read_fifo_dma_descriptor,DMAC_BTCTRL_BLOCKACT_SUSPEND_Val);						//DMA_BLOCK_ACTION_SUSPEND		
	
	// read fifo registers
	setup_transfer_descriptor(DMA_TRANSFER_DESCRIPTOR_TX,&spi_read_fifo_dma_descriptor,rd_pdram_start,rd_pdram_lenght,&spi_read2ram_dma_descriptor,DMAC_BTCTRL_BLOCKACT_NOACT_Val);								//DMA_BLOCK_ACTION_NOACT
	setup_transfer_descriptor(DMA_TRANSFER_DESCRIPTOR_TX_DUMMY,&spi_read2ram_dma_descriptor,0,4*read_var_num,&spi_write_fifo_dma_descriptor,DMAC_BTCTRL_BLOCKACT_SUSPEND_Val);									//DMA_BLOCK_ACTION_SUSPEND
	
	//// RX DESCRIPTORS
	
	// abort fifo
	setup_transfer_descriptor(DMA_TRANSFER_DESCRIPTOR_RX_DUMMY,&spi_rx_abort_fifo_dma_descriptor,0,cf_pdram_lenght,&spi_rx_clear_rd_fifo_dma_descriptor,DMAC_BTCTRL_BLOCKACT_SUSPEND_Val);						//DMA_BLOCK_ACTION_SUSPEND
	
	//clear initial read data from LAN9252 to avoid errors.
	setup_transfer_descriptor(DMA_TRANSFER_DESCRIPTOR_RX_DUMMY,&spi_rx_clear_rd_fifo_dma_descriptor,0,wr_pdram_lenght+4*write_var_num,&spi_rx_write_cl_fifo_dma_descriptor,DMAC_BTCTRL_BLOCKACT_SUSPEND_Val);	//DMA_BLOCK_ACTION_SUSPEND
	setup_transfer_descriptor(DMA_TRANSFER_DESCRIPTOR_RX_DUMMY,&spi_rx_write_cl_fifo_dma_descriptor,0,cl_pdram_lenght,&spi_rx_write_fifo_dma_descriptor,DMAC_BTCTRL_BLOCKACT_SUSPEND_Val);						//DMA_BLOCK_ACTION_SUSPEND
	
	// write fifo registers , dummy receive
	setup_transfer_descriptor(DMA_TRANSFER_DESCRIPTOR_RX_DUMMY,&spi_rx_write_fifo_dma_descriptor,0,spi_head+4*write_var_num,&spi_rx_config_fifo_dma_descriptor,DMAC_BTCTRL_BLOCKACT_SUSPEND_Val);				//DMA_BLOCK_ACTION_SUSPEND
	
	// write fifo config. dummy receive
	setup_transfer_descriptor(DMA_TRANSFER_DESCRIPTOR_RX_DUMMY,&spi_rx_config_fifo_dma_descriptor,0,cf_pdram_lenght,&spi_rx_read_fifo_dma_descriptor,DMAC_BTCTRL_BLOCKACT_SUSPEND_Val);							//DMA_BLOCK_ACTION_SUSPEND
	
	// read fifo registers
	setup_transfer_descriptor(DMA_TRANSFER_DESCRIPTOR_RX_DUMMY,&spi_rx_read_fifo_dma_descriptor,0,spi_head,&spi_rx_read2ram_dma_descriptor,DMAC_BTCTRL_BLOCKACT_NOACT_Val);										//DMA_BLOCK_ACTION_NOACT
	setup_transfer_descriptor(DMA_TRANSFER_DESCRIPTOR_RX,&spi_rx_read2ram_dma_descriptor,&ram_buffer[ram_rd_start],4*read_var_num,&spi_rx_write_fifo_dma_descriptor,DMAC_BTCTRL_BLOCKACT_SUSPEND_Val);			//DMA_BLOCK_ACTION_SUSPEND

	// ----------------------------------------------------------------------
	// Register Callbacks
	// ---------------------------------------------------------------------- 
	spi_m_dma_register_callback(&SPI_0, SPI_DEV_CB_DMA_SUSPEND, LAN9252_rx_susp);
	
	// ----------------------------------------------------------------------
	// Link Descriptors
	// ----------------------------------------------------------------------
	//_descriptor_section[1] = spi_abort_fifo_dma_descriptor;
	_dma_set_descriptor(0, spi_rx_abort_fifo_dma_descriptor);
	_dma_set_descriptor(1, spi_abort_fifo_dma_descriptor);

	// ----------------------------------------------------------------------
	// Enable SPI DMA
	// ----------------------------------------------------------------------
	spi_m_dma_enable(&SPI_0);
	gpio_set_pin_level(ECAT_SPI_CS_PIN, false);
	_dma_enable_transaction(0,false);
	_dma_enable_transaction(1,false);
	//DMAC->Channel[0].CHCTRLA.bit.ENABLE = true;
	//DMAC->Channel[1].CHCTRLA.bit.ENABLE = true;
	
}

static void setup_transfer_descriptor(enum dma_transfer_descriptor_type type, DmacDescriptor *descriptor, const uint32_t start, 
										const uint32_t lenght, const uint32_t next_descriptor, uint16_t block_action)
{
		
	/*** DMA RX Descriptor Initialization step by step
	 *** 1- Validate the Descriptor
	 *** 2- Event Output = DMA_EVENT_OUTPUT_BEAT
	 *** 3- block_action = DMA_BLOCK_ACTION_NOACT;
	 *** 4- beat_size = DMA_BEAT_SIZE_BYTE;
	 *** 5- src_increment_enable = false;
	 *** 6- dst_increment_enable = false;
	 *** 7- step_selection = DMA_STEPSEL_DST;
	 *** 8- step_size = DMA_ADDRESS_INCREMENT_STEP_SIZE_1;
	 *** 9- block_transfer_count;
	 *** 10- source_address = (uint32_t)NULL;
	 *** 11- destination_address = (uint32_t)NULL;
	 *** 12- next_descriptor_address = 0;
	 ***/
		
	hri_dmacdescriptor_set_BTCTRL_VALID_bit(descriptor);										// 1.Validate the Descriptor
	hri_dmacdescriptor_write_BTCTRL_EVOSEL_bf(descriptor, DMAC_BTCTRL_EVOSEL_BURST_Val);		// 2. Define mentions 0x3 as Burst, but 0x3 = BEAT (Event strobe when beat transfer complete)
	hri_dmacdescriptor_write_BTCTRL_BLOCKACT_bf(descriptor, block_action);						// 3. block_action
	hri_dmacdescriptor_write_BTCTRL_BEATSIZE_bf(descriptor, DMAC_BTCTRL_BEATSIZE_BYTE_Val);		// 4. beat_size
		
	switch (type){
		case DMA_TRANSFER_DESCRIPTOR_TX:
			hri_dmacdescriptor_write_BTCTRL_SRCINC_bit(descriptor, true);						// 5. src_increment_enable = true
			hri_dmacdescriptor_write_BTCTRL_DSTINC_bit(descriptor, false);						// 6. dst_increment_enable = false
			hri_dmacdescriptor_write_SRCADDR_SRCADDR_bf(descriptor, start+lenght);				// 10. source_address
			hri_dmacdescriptor_write_DSTADDR_DSTADDR_bf(descriptor, (uint32_t)(&SPI_LAN9252->SPI.DATA.reg));	// 11. destination_address
		break;	
		case DMA_TRANSFER_DESCRIPTOR_TX_DUMMY:
			hri_dmacdescriptor_write_BTCTRL_SRCINC_bit(descriptor, false);						// 5. src_increment_enable = false
			hri_dmacdescriptor_write_BTCTRL_DSTINC_bit(descriptor, false);						// 6. dst_increment_enable = false
			hri_dmacdescriptor_write_SRCADDR_SRCADDR_bf(descriptor, dummy_register);			// 10. source_address
			hri_dmacdescriptor_write_DSTADDR_DSTADDR_bf(descriptor, (uint32_t)(&SPI_LAN9252->SPI.DATA.reg));	// 11. destination_address
		break;
		case DMA_TRANSFER_DESCRIPTOR_RX:
			hri_dmacdescriptor_write_BTCTRL_SRCINC_bit(descriptor, false);						// 5. src_increment_enable = false
			hri_dmacdescriptor_write_BTCTRL_DSTINC_bit(descriptor, true);						// 6. dst_increment_enable = true
			hri_dmacdescriptor_write_SRCADDR_SRCADDR_bf(descriptor, (uint32_t)(&SPI_LAN9252->SPI.DATA.reg));	// 10. source_address
			hri_dmacdescriptor_write_DSTADDR_DSTADDR_bf(descriptor, start+lenght);				// 11. destination_address
		break;
		case DMA_TRANSFER_DESCRIPTOR_RX_DUMMY:
			hri_dmacdescriptor_write_BTCTRL_SRCINC_bit(descriptor, false);						// 5. src_increment_enable
			hri_dmacdescriptor_write_BTCTRL_DSTINC_bit(descriptor, false);						// 6. dst_increment_enable = false;
			hri_dmacdescriptor_write_SRCADDR_SRCADDR_bf(descriptor, (uint32_t)(&SPI_LAN9252->SPI.DATA.reg)); // 10. source_address
			hri_dmacdescriptor_write_DSTADDR_DSTADDR_bf(descriptor, &spi_rx_buffer[0]);			// 11. destination_address
		break;		
	}
	
	hri_dmacdescriptor_write_BTCTRL_STEPSEL_bit(descriptor, false);								// 7. step_selection = DMA_STEPSEL_DST
	hri_dmacdescriptor_write_BTCTRL_STEPSIZE_bf(descriptor, DMAC_BTCTRL_STEPSIZE_X1_Val);		// 8. step_size = DMA_ADDRESS_INCREMENT_STEP_SIZE_1
	hri_dmacdescriptor_write_BTCNT_BTCNT_bf(descriptor, lenght);								// 9. block_transfer_count;
	hri_dmacdescriptor_write_DESCADDR_DESCADDR_bf(descriptor, next_descriptor);					// 12. next_descriptor_address = 0;
}