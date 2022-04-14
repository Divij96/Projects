#include <atmel_start.h>
#include "driver_examples.h"
#include "driver_init.h"
#include "utils.h"
#include "helper.h"
#include <stdbool.h>
#include <hal_atomic.h>

//
#define COMPARE_CHANNEL_0 0
#define ADC0_CHANNEL 0
#define ADC1_CHANNEL 0
#define CURRENT_MES_FLAG 0
#define TORQUE_OFFSET 1.65 //in Volt //measure this value with voltmeter when no force is applied and set it in macro Default is VDD/2
#define TORQUE_REF 3.3 // in Volt torque reference voltage
#define G_AMP 561 //scalar
#define G_SENS 0.5 // mV/V
//QSPI_Definitions

struct data_send_ethercat{
	uint16_t status;
	uint16_t mode;
	float rel_position;
	int16_t rel_revolution;
	uint16_t Motor_rel_Revolution;
	float Motor_rel_position;
	uint32_t Motor_abs_position;
	float joint_speed;
	float motor_speed;
	float tau;
	float tau_D;
	float i_abs;
	float i_q;
	float i_d;
	float qw;
	float qx;
	float qy;
	float qz;	
	};
	
struct data_receive_ethercat{
	uint16_t mode;
	uint16_t set;
	float desired_position;
	float offset_position;
	float desired_speed;
	float max_speed;
	float tau_desired;
	float tau_max;
	float tau_kp;
	float tau_kd;
	uint16_t tau_N;
	uint16_t tau_gain;
	float tau_offset;
	uint32_t tau_filter;
	float I_desired;
	float I_max;
	float Iq_kp;
	float IqKi;
	
	};	
uint32_t abort_FIFO[3]   = {0x40000000,   0x00, 0x40000000}; // Abort FiFo
uint8_t clear_RDRAM[2] =  {0x00401000, 0x80000000}; //Clear RDRAM
uint32_t zero_32[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
struct data_send_ethercat data_send_ethercat_zero;

uint32_t data_send[16] = {0x00000000,
		0x00000001,
		0x00000002,
		0x00000004,
		0x00000008,
		0x00000010,
		0x00000020,
		0x00000040,
		0x00000080,
		0x00000100,
		0x00000400,
		0x00008000,
		0x00010000,
		0x00200000,
		0x04000000,
	    0x80000000};	
		
uint32_t config_fifo[4] = {0x00401000, 0x80000000, 0x00401800, 0x80000000};
uint32_t received_data[16];
uint8_t msg[4] = {0x01, 0x02, 0x03, 0x04};
			
//ADC_definitions
uint16_t buffA, buffB, buffC;
float currA, currB, voltA, voltB;
float curr_reference = 0;
float V_in_ADC, V_sens, trq_in_N;

//CCL definitions
volatile uint32_t H_0_pulse_width;
float rot_speed = 0;
float H_0_pulse_width_in_sec;

//PWM and Speed calc
uint8_t hall = 0;
uint8_t steps[8] = {0,5,3,4,1,0,2,0};
uint16_t patterns[16] = {0x00FF, 0x10D2, 0x40E4, 0x40D2, 0x2036, 0x1036, 0x20E4, 0x00FF, 0x00FF, 0x20E4, 0x1036, 0x2036, 0x40D2, 0x40E4, 0x10D2, 0x00FF};
uint8_t step[2] = {0};
int32_t revolution_counter_elec = 0;
float revolution_counter_mech = 0;
float rel_position = 0;


//----CONTROL_VARIABLES--------
volatile float out_i[4]={0};
volatile float out_w[4]={0};
volatile float out_p[4]={0};
volatile bool current_ready, trq_ready = {true, true};
float desired_pos = 0;
float desired_trq = 0;
float desired_spd = 0;
float desired_curr = 0;
float kp_p = 0; //position prop gain
float kp_n = 0; //speed prop gain
float joint_max_speed = 0;
float v_max = 24; //max voltage
float i_max = 0.3; //max current
float w_max = 700; //max current
float kp_i = 0; //curr prop gain
float ki_i = 0; //curr int gain

float kt = 0.0322; //  Nm/A
uint8_t direction = 0;
uint8_t activate_control = 1;
uint8_t control_type = 0; //0 curr, 1 speed, 2 pos

float volt_ref = 0;
float curr_ref_controller = 0;
float speed_ref_controller = 0;

//----for encoder
extern uint8_t encoder_raw_data[6];
extern uint16_t encoder_multiturn_counter;
extern float encoder_position;
extern float encoder_position_in_rad;
extern float encoder_offset;
extern uint8_t encoder_Err;
extern uint8_t encoder_Not_Warning ;
extern uint8_t encoder_crc_inv;
struct data_send_ethercat data_send_ethercat_var;
struct data_receive_ethercat data_receive_ethercat_var;


///Interrupt Handlers
void ADC0_1_Handler(){ //ADC0_RESRDY
	ADC0->INTFLAG.bit.RESRDY=1;
		if(!current_ready || !trq_ready){
			if (ADC0->INPUTCTRL.bit.MUXPOS == 0) {
				buffA= ADC0->RESULT.reg;
				buffB= ADC1->RESULT.reg;

				voltA= (buffA/4095.0) * 3300; //Define the conversion ratio of your ADC value to a float representing the current
				voltB= (buffB/4095.0) * 3330;
	
				currA=(voltA-1500)/200;
				currB=(voltB-1500)/200;
				current_ready = true;
				ADC0->INPUTCTRL.bit.MUXPOS = 1; //Change channel to 1 for torque read
				ADC0->SWTRIG.bit.START = 1; //Start conversion manually, cleared automatically in HW
			}
			else if (ADC0->INPUTCTRL.bit.MUXPOS == 1) {
				buffC= ADC0->RESULT.reg; //Read out trq
				V_in_ADC = (buffA*TORQUE_REF)/4095;
				V_sens = (V_in_ADC - TORQUE_OFFSET)/G_AMP;
				trq_in_N = V_sens / (G_SENS*TORQUE_REF);
				trq_ready = true;
				ADC0->INPUTCTRL.bit.MUXPOS = 0; //Change channel to 0 for following current read
			}
		}
}



void TC0_Handler(void)
{
	if(TC0->COUNT32.INTFLAG.bit.OVF){ // if overflow happened
		TC0->COUNT32.INTFLAG.bit.OVF = 1;
		rot_speed = 0x0;
	}
	else if  (TC0->COUNT32.INTFLAG.bit.ERR){
		TC0->COUNT32.INTFLAG.bit.ERR = 1;
		rot_speed = 0xFFFFFFFF;
	}
	else{
		TC0->COUNT32.INTFLAG.bit.MC0 = 1;
		H_0_pulse_width = TC0->COUNT32.CC[0].reg;
		H_0_pulse_width_in_sec = H_0_pulse_width * (float) 0.00000001; // 100 Mhz
		rot_speed = (3.1415 / 3.0 / 7) / H_0_pulse_width_in_sec; // omega = (60° / 7 / delta_t)  - 7 because of pole pairs of the motor
	}
	
}

void set_motor_voltage(uint8_t hall)
{
/////
	TCC0->PATTBUF.reg = patterns[hall];
	//hri_tcc_write_CTRLA_ENABLE_bit(TCC0, 1 << TCC_CTRLA_ENABLE_Pos); /* Enable: enabled */

		}

void get_position(uint8_t hall)
{
	step[0] = steps[hall]; //update position

	if (step[0] == 5 && step[1] == 0)
	revolution_counter_elec++;
	else if (step[0] == 0 && step[1] == 5)
	revolution_counter_elec--;

	rel_position = revolution_counter_elec*5 + step[1];
	revolution_counter_mech = rel_position / 42;

	step[1] = step[0];  //step[1] = previous position, step[0] - latest position
}





int main(void)
{
	
	volatile uint16_t  *status				=&ram_buffer[ram_wr_start];
	volatile uint16_t  *run_mode			=(((uint16_t *)&ram_buffer[ram_wr_start])+1);
	volatile uint32_t *read_joint_position	=&ram_buffer[ram_wr_start+1];
	volatile int16_t   *read_j_revolution	=&ram_buffer[ram_wr_start+2];
	volatile uint16_t  *read_revolution		=(((uint16_t *)&ram_buffer[ram_wr_start+2])+1);
	volatile float  *read_rel_position	=&ram_buffer[ram_wr_start+3];
	volatile uint32_t *read_position		=&ram_buffer[ram_wr_start+4];
	volatile float  *read_Joint_speed		=&ram_buffer[ram_wr_start+5];
	volatile float  *read_speed			=&ram_buffer[ram_wr_start+6];
	volatile float  *read_torque			=&ram_buffer[ram_wr_start+7];
	volatile float  *read_torque_D		=&ram_buffer[ram_wr_start+8];
	volatile float  *read_current_i_m		=&ram_buffer[ram_wr_start+9];
	volatile float  *read_current_i_q		=&ram_buffer[ram_wr_start+10];
	volatile float  *read_current_i_d		=&ram_buffer[ram_wr_start+11];
	volatile float  *IMU_qw				=&ram_buffer[ram_wr_start+12];
	volatile float  *IMU_qx				=&ram_buffer[ram_wr_start+13];
	volatile float  *IMU_qy				=&ram_buffer[ram_wr_start+14];
	volatile float  *IMU_qz				=&ram_buffer[ram_wr_start+15];

	//read
	volatile	uint16_t *control_mode			=&ram_buffer[ram_rd_start];
	static		uint16_t *control_set			=(((uint16_t *)&ram_buffer[ram_rd_start])+1);
	static		int32_t *desired_position		=&ram_buffer[ram_rd_start+1];
	static		uint32_t *Motor_position_offset	=&ram_buffer[ram_rd_start+2];
	static		float *desired_speed			=&ram_buffer[ram_rd_start+3];
	static		float *Joint_max_speed			=&ram_buffer[ram_rd_start+4];
	static		float *desired_torque			=&ram_buffer[ram_rd_start+5];
	static		float *tau_max					=&ram_buffer[ram_rd_start+6];
	static		float *tau_kp					=&ram_buffer[ram_rd_start+7];
	static		float *tau_kd					=&ram_buffer[ram_rd_start+8];
	static		uint16_t *tau_N					=&ram_buffer[ram_rd_start+9];
	static		uint16_t *tau_Gain				=(((uint16_t *)&ram_buffer[ram_rd_start+9])+1);
	static		float *tau_offset				=&ram_buffer[ram_rd_start+10];
	volatile	uint32_t *tau_filter			=&ram_buffer[ram_rd_start+11];
	static		float *I_desired				=&ram_buffer[ram_rd_start+12];
	static		float *I_max					=&ram_buffer[ram_rd_start+13];
	volatile	float *i_kp						=&ram_buffer[ram_rd_start+14];
	volatile	float *i_ki						=&ram_buffer[ram_rd_start+15];
	/* Initializes MCU, drivers and middleware */
	data_send_ethercat_zero.status = 0;
	data_send_ethercat_zero.mode = 0;
	data_send_ethercat_zero.rel_position= 0;
	data_send_ethercat_zero.rel_revolution= 0;
	data_send_ethercat_zero.Motor_rel_Revolution= 0;
	data_send_ethercat_zero.Motor_rel_position= 0;
	data_send_ethercat_zero.Motor_abs_position= 0;
	data_send_ethercat_zero.joint_speed= 0;
	data_send_ethercat_zero.motor_speed= 0;
	data_send_ethercat_zero.tau= 0;
	data_send_ethercat_zero.tau_D= 0;
	data_send_ethercat_zero.i_abs= 0;
	data_send_ethercat_zero.i_q= 0;
	data_send_ethercat_zero.i_d= 0;
	data_send_ethercat_zero.qw= 0;
	data_send_ethercat_zero.qx= 0;
	data_send_ethercat_zero.qy= 0;
	data_send_ethercat_zero.qz= 0;
	
	
	data_send_ethercat_var.status = 1;
	data_send_ethercat_var.mode = 2;
	data_send_ethercat_var.rel_position= 3.0;
	data_send_ethercat_var.rel_revolution= 4;
	data_send_ethercat_var.Motor_rel_Revolution= 5;
	data_send_ethercat_var.Motor_rel_position= 6.0;
	data_send_ethercat_var.Motor_abs_position= 7;
	data_send_ethercat_var.joint_speed= 8.0;
	data_send_ethercat_var.motor_speed= 9.0;
	data_send_ethercat_var.tau= 10.0;
	data_send_ethercat_var.tau_D= 11.0;
	data_send_ethercat_var.i_abs= 12.0;
	data_send_ethercat_var.i_q= 13.0;
	data_send_ethercat_var.i_d= 14.0;
	data_send_ethercat_var.qw= 15.0;
	data_send_ethercat_var.qx= 16.0;
	data_send_ethercat_var.qy= 17.0;
	data_send_ethercat_var.qz= 18.0;


	atmel_start_init();
	custom_logic_init();
	custom_logic_enable();
	qspi_sync_enable(&ECAT_QSPI);
	adc_sync_enable_channel(&CURR_A_TRQ_ADC, 0);
	adc_sync_enable_channel(&CURR_B_ADC, 0);
	//QUAD_SPI_0_write(addr_AbortFIFO, AbortFIFO, sizeof(AbortFIFO));
	//QUAD_SPI_0_write(addr_ClearRdRAM_1, ClearRdRAM_1, sizeof(ClearRdRAM_1));
	//QUAD_SPI_0_write(addr_ClearRdRAM_2, ClearRdRAM_2, sizeof(ClearRdRAM_2));
		
	NVIC_EnableIRQ(TC0_IRQn);
	NVIC_EnableIRQ(ADC0_1_IRQn);
	ADC0->INTENSET.bit.RESRDY = 1; //Reset the RESRDY Interrupt
		
	gpio_set_pin_level(ENC_CS, 1); //setup CS SPI_Encoder
	ENC_SPI_init();
	ENC_SPI_enable();
	set_encoder_offset_zero();
				ECAT_QSPI_send(&abort_FIFO,12, 0x02, 0x430C);
				ECAT_QSPI_send(&zero_32,64, 0x02, 0x0020);
				ECAT_QSPI_send(&clear_RDRAM,16, 0x02, 0x4310 );

	uint32_t hallA, hallB, hallC;
		while (1) {
			receive_data_from_encoder();

			// 		kp_i = ;
			// 		ki_i = ;
			// 		desired_curr = ;
			// 		desired_spd = *desired_speed;
			// 		desired_pos = *desired_position;
			// 		desired_trq = *desired_torque;
			// 		kp_n = *tau_kd;
			// 		kp_p = *tau_kp;
			// 		i_max = *I_max;
			// 		w_max = *Joint_max_speed;
			// 		activate_control = *control_mode;
			// 		control_type = *control_set;
			
			if(activate_control){
				
				if (current_ready && trq_ready){
					
					current_ready = false;
					trq_ready = false;
					hall = 0;
					hallA = gpio_get_pin_level(Hall_A);
					hallB = gpio_get_pin_level(Hall_B);
					hallC = gpio_get_pin_level(Hall_C);

					hall = (hallA<<2)|(hallB<<1)|hallC;

					switch(hall) {
						case 1:
						curr_reference = -currA;
						break;
						case 2:
						curr_reference = -currB;
						break;
						case 3:
						curr_reference = -currA;
						break;
						case 4:
						curr_reference = currB;
						break;
						case 5:
						curr_reference = currB;
						break;
						case 6:
						curr_reference = -currA;
						break;
					}
					
					get_position(hall);
					
					if(control_type == 1){ //speed control
						p_controller(out_w, desired_spd, rot_speed, kp_n, i_max*kt); // max current = 0.3
						curr_ref_controller =  out_w[0] / kt;
						pi_controller(out_i, curr_ref_controller, curr_reference, kp_i, ki_i, v_max); //max voltage = 24
					}
					else if (control_type == 2){ //position control
						p_controller(out_p, desired_pos, rel_position, kp_p, w_max); //max speed = 700
						p_controller(out_w, out_p[0], rot_speed, kp_n, i_max*kt); // max current = 0.3
						curr_ref_controller =  out_w[0] / kt;
						pi_controller(out_i, curr_ref_controller, curr_reference, kp_i, ki_i, v_max); //max voltage = 24
					}
					else{ //current control
						pi_controller(out_i, desired_curr, curr_reference, kp_i, ki_i, v_max); //max voltage = 24
					}
					
					volt_ref = (out_i[0] + kt*rot_speed) * 2000 / 24; //Feed-forward
					direction = (volt_ref <0);
					
					hall = hall | (direction <<3);
					
					set_motor_voltage(hall);
					TCC0->CCBUF->reg = 0; /* Channel 0 Compare/Capture Value: 0x0 */
					

					ECAT_QSPI_send(&data_send_ethercat_var,64, 0x02, 0x0020);
					ECAT_QSPI_send(&config_fifo,16, 0x02, 0x4308);
					ECAT_QSPI_read(&data_receive_ethercat_var, 64, 0x03, 0x030000);
					data_send[0] += 1;
					// 				if(volt_ref > 0){
					// 					TCC0->CCBUF->reg = volt_ref;
					// 				}
					// 				else{
					// 					TCC0->CCBUF->reg = -volt_ref;
					// 				}
					
					// 				*read_position = step[1];
					// 				*read_rel_position = rel_position;
					// 				*read_current_i_q = currA_in_Amp;
					// 				*read_current_i_d = CurrB_in_Amp;
					// 				*read_current_i_m = curr_reference;
					// 				*read_speed = rot_speed;
				}
			}
			else{ //control deactivated
				TCC0->CCBUF->reg = 0; //turn off pwm
			}
			
			
		}			//end while
	}				//end main

