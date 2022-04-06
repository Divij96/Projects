/*
 * ethercat_e54.h
 *
 * Created: 02/03/2021 10:49:33
 *  Author: Nick-XMG
 */ 

#ifndef ETHERCAT_E54_H_
#define ETHERCAT_E54_H_

// ----------------------------------------------------------------------
// Header Files
// ----------------------------------------------------------------------
// ----------------------------------------------------------------------
// Defines
// ----------------------------------------------------------------------


// ----------------------------------------------------------------------
// Types
// ----------------------------------------------------------------------

enum dma_transfer_descriptor_type {
	DMA_TRANSFER_DESCRIPTOR_TX,
	DMA_TRANSFER_DESCRIPTOR_RX,
	DMA_TRANSFER_DESCRIPTOR_TX_DUMMY,
	DMA_TRANSFER_DESCRIPTOR_RX_DUMMY,
};


// ----------------------------------------------------------------------
// Register Addresses
// ----------------------------------------------------------------------
#define read_var_num 16  //to change to 16bits need to change ecat spi lenght.
#define write_var_num 16 //max 64
// #define read_var_num 2  //to change to 16bits need to change ecat spi lenght.
// #define write_var_num 2 //max 64

#define ram_wr_start 0
#define ram_rd_start write_var_num //write_var_num
#define ram_real_wr_start  ram_wr_start

#define SPI_READ   0x03
#define SPI_WRITE  0x02
#define SPI_INC    0x40
#define SPI_DEC	   0x80
#define TEST_VAL   0x87654321
#define CSR_BUSY   0x80000000
#define ADDR_BYTES 2
#define CSR_READ   1<<30
#define CSR_WRITE  0<<30
#define CSR_SIZE   4<<16
#define CSR_HW_RD  1<<27

#define ECAT_PRAM_RD_DATA		0x0000
#define ECAT_PRAM_WR_DATA		0x2000
#define ID_REV					0x5000
#define IRQ_CFG					0x5400
#define INT_STS					0x5800
#define INT_EN					0x5C00
#define BYTE_TEST				0x6400
#define HW_CFG					0x7400
#define PMT_CTRL				0x8400
#define GPT_CFG					0x8C00
#define GPT_CNT					0x9000
#define FREE_RUN				0x9C00
#define RESET_CTL				0xF801
#define ECAT_CSR_DATA			0x0003
#define ECAT_CSR_CMD			0x0403
#define ECAT_PRAM_RD_ADDR_LEN	0x0803
#define ECAT_PRAM_RD_CMD		0x0C03
#define ECAT_PRAM_WR_ADDR_LEN	0x1003
#define ECAT_PRAM_WR_CMD		0x1403

#define PDRAM_RD_ADDRESS		0x1100
#define PDRAM_WR_ADDRESS		0x1800

#define PDRAM_RD_LENGTH			4*read_var_num
#define PDRAM_WR_LENGTH			4*write_var_num

// ----------------------------------------------------------------------
// SPI 
// ----------------------------------------------------------------------

#define spi_head 3
#define hw_tst_length			spi_head +4
#define by_tst_length			spi_head +4
#define rd_status_length		spi_head +4*4
#define abort_fifo_length		spi_head +4*4
#define cf_pdram_lenght			spi_head +4*4
#define rd_pdram_lenght			spi_head
#define wr_pdram_lenght			spi_head
#define cl_pdram_lenght			spi_head +4*2

#define rd_fifo_lenght
#define wr_fifo_lenght


// ----------------------------------------------------------------------
// Flags
// ----------------------------------------------------------------------

volatile bool tx_ethercat;
volatile bool tx_ethercat_done;

// ----------------------------------------------------------------------
// DMA descriptors
// ----------------------------------------------------------------------
COMPILER_ALIGNED(16)
DmacDescriptor spi_config_fifo_dma_descriptor;
DmacDescriptor spi_read_fifo_dma_descriptor;
DmacDescriptor spi_write_fifo_dma_descriptor;
DmacDescriptor spi_read2ram_dma_descriptor;
DmacDescriptor spi_write2ram_dma_descriptor;
DmacDescriptor spi_abort_fifo_dma_descriptor;
DmacDescriptor spi_clear_rd_fifo_dma_descriptor;
DmacDescriptor spi_write_cl_fifo_dma_descriptor;

DmacDescriptor spi_rx_config_fifo_dma_descriptor;
DmacDescriptor spi_rx_read_fifo_dma_descriptor;
DmacDescriptor spi_rx_write_fifo_dma_descriptor;
DmacDescriptor spi_rx_read2ram_dma_descriptor;
DmacDescriptor spi_rx_abort_fifo_dma_descriptor;
DmacDescriptor spi_rx_clear_rd_fifo_dma_descriptor;
DmacDescriptor spi_rx_write_cl_fifo_dma_descriptor;

// ----------------------------------------------------------------------
// Global Variables
// ----------------------------------------------------------------------
volatile uint32_t ram_buffer[(read_var_num+write_var_num)];

volatile  uint8_t spi_rx_buffer[30];							

static uint8_t spi_tx_buffer[110] = {
	SPI_READ,HW_CFG,HW_CFG>>8,0xFF,0xFF,0xFF,0xFF,
	SPI_READ,BYTE_TEST,BYTE_TEST>>8,0xFF,0xFF,0xFF,0xFF,
	SPI_WRITE,ECAT_PRAM_RD_ADDR_LEN | SPI_INC, (ECAT_PRAM_RD_ADDR_LEN | SPI_INC)>>8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x40,
	SPI_READ, ECAT_PRAM_RD_ADDR_LEN | SPI_INC, (ECAT_PRAM_RD_ADDR_LEN | SPI_INC)>>8,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	SPI_WRITE,ECAT_PRAM_RD_ADDR_LEN | SPI_INC, (ECAT_PRAM_RD_ADDR_LEN | SPI_INC)>>8,PDRAM_RD_ADDRESS,(PDRAM_RD_ADDRESS>>8),PDRAM_RD_LENGTH,PDRAM_RD_LENGTH>>8,0x00,0x00,0x00,CSR_BUSY>>24,PDRAM_WR_ADDRESS,PDRAM_WR_ADDRESS>>8,PDRAM_WR_LENGTH,PDRAM_WR_LENGTH>>8,0x00,0x00,0x00,CSR_BUSY>>24,
	SPI_READ,ECAT_PRAM_RD_DATA,(ECAT_PRAM_RD_DATA)>>8, //modification for FIFO FIXED
	SPI_WRITE,ECAT_PRAM_WR_DATA,(ECAT_PRAM_WR_DATA)>>8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	SPI_WRITE,ECAT_PRAM_WR_ADDR_LEN | SPI_INC, (ECAT_PRAM_WR_ADDR_LEN | SPI_INC)>>8,PDRAM_RD_ADDRESS,(PDRAM_RD_ADDRESS>>8),PDRAM_RD_LENGTH,PDRAM_RD_LENGTH>>8,0x00,0x00,0x00,CSR_BUSY>>24};


const static uint32_t dummy_register = &spi_tx_buffer[4];
const static uint32_t hw_tst_start =   &spi_tx_buffer[0];
const static uint32_t by_tst_start =   &spi_tx_buffer[0+hw_tst_length];
const static uint32_t abort_fifo_start=&spi_tx_buffer[0+hw_tst_length+by_tst_length];
const static uint32_t rd_status_start= &spi_tx_buffer[0+hw_tst_length+by_tst_length+abort_fifo_length];
const static uint32_t cf_pdram_start = &spi_tx_buffer[0+hw_tst_length+by_tst_length+abort_fifo_length+rd_status_length];
const static uint32_t rd_pdram_start = &spi_tx_buffer[0+hw_tst_length+by_tst_length+abort_fifo_length+rd_status_length+cf_pdram_lenght];
const static uint32_t wr_pdram_start = &spi_tx_buffer[0+hw_tst_length+by_tst_length+abort_fifo_length+rd_status_length+cf_pdram_lenght+rd_pdram_lenght];
volatile static uint32_t cl_pdram_start = &spi_tx_buffer[0+hw_tst_length+by_tst_length+abort_fifo_length+rd_status_length+cf_pdram_lenght+rd_pdram_lenght+wr_pdram_lenght+write_var_num];
//
//
const static uint32_t *spi_rx_reg   = &spi_rx_buffer[0];

// ----------------------------------------------------------------------
// 	Driver Instances
// ----------------------------------------------------------------------
//static struct timer_task tc_syncro_task;
#define SYNC_1kHz_MODULE	TC7 
#define SPI_LAN9252			SERCOM5
// ----------------------------------------------------------------------
// 	Callbacks
// ----------------------------------------------------------------------
// 	External
extern void One_ms_cycle_callback(void);

// 	Internal
static void transfer_error(struct _dma_resource *resource);
static void LAN9252_rx_done(struct _dma_resource *resource);
static void LAN9252_rx_susp(struct _dma_resource *resource);

// ----------------------------------------------------------------------
// 	Interrupt Handlers
// ----------------------------------------------------------------------
//extern void TC7_Handler(void);

// ----------------------------------------------------------------------
// 	functions
// ----------------------------------------------------------------------
void ethercat_update(void);
void config_ethercat_tc(void);
void config_ethercat_sercom(void);
void configure_dma_resource(void);
void configure_ethercat_dma_descriptors(void);
static void setup_transfer_descriptor(enum dma_transfer_descriptor_type type, DmacDescriptor *descriptor,
								 const uint32_t start, const uint32_t lenght, const uint32_t next_descriptor, uint16_t block_action);
	

#endif /* ETHERCAT_E54_H_ */
