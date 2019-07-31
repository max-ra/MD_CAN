//#if defined(__MCP2515__)
/*
 * MD_MCP2515.c
 *
 * Created: 29.07.2019 15:07:51
 */ 
// ##### Including #####
#include "MCP2515_CAN.h"
#include "../../../../02_kernel/scheduler/message_query.h"
#include "../../../../02_kernel/scheduler/SoftwareTimer_SFT.h"
#include "../../../MD_IO/MD_IO.h"
#include "../../../MD_SPI/MD_SPI.h"
#include "../../MD_CAN.h"

// ##### Settings #####


// ##### definition #####


// ##### Status definition #####


// ##### variable #####
Task_Status_Byte MD_MCP2515_Status;

//init
static uint_fast8_t Init_worker_step;

//Timer
SFT_Timer MCP_Wakeup_Timer;

//SPI link
MD_SPI_MSQ_Type SPI_Sender;
uint_fast8_t SPI_Done;
uint_fast8_t SPI_Buffer[MCP2515_CAN_setting_max_spi_buffer_byte];

// ##### function #####
void MCP2515_CAN_wakeup (uint_fast8_t temp);
void MCP2515_CAN_SPI_callback(uint_fast8_t status);
void next_init_step(void);

void CAN_init(void) {
	//setup Hardware
	DDRD |= (1<<PIND2);
	MD_IO_SPI_CAN_Select_init();
	
	//Setup SPI link
	SPI_Sender.callback_SPI = &MCP2515_CAN_SPI_callback;
	SPI_Sender.DataPointer = (uint_fast8_t*)&SPI_Buffer;
	SPI_Sender.Direction = SPI_write;
	SPI_Sender.size = 1;
	MCP2515_CAN_SPI_callback(SPI_stop);
	
	//Setup Timer
	MCP_Wakeup_Timer.function_parameter = 0;
	MCP_Wakeup_Timer.time_ms = 10;
	MCP_Wakeup_Timer.time_out_function = &MCP2515_CAN_wakeup;
	
	Init_worker_step = 0;
}

void MCP2515_CAN_wakeup(uint_fast8_t temp) {
	//set stet at init
	Init_worker_step = temp;
}

void MCP2515_CAN_SPI_callback(uint_fast8_t status) {
	if (status == SPI_start) {
		PORTD &= ~(1<<PIND2);
		MD_IO_SPI_CAN_Select_off();
	} else if (status == SPI_stop) {
		next_init_step();
		PORTD |= (1<<PIND2);
		MD_IO_SPI_CAN_Select_on();
	}
}


void MD_CAN_Init_worker(void) {
	asm("nop");
	switch (Init_worker_step) {
		
		case 0: //reset hardware
			SPI_Buffer[0] = SPI_RESET;
			SPI_Sender.Direction = SPI_write;
			SPI_Sender.size = 1;
			MSQ_message_push(&MD_SPI_MSQ_queue, &SPI_Sender);
			Init_worker_step++;
		break;
		
		case 1:
			//do nothing wile mcp2515 is reseted
			asm("nop");
		break;
		
		case 2:
			//delay 10ms
			MCP_Wakeup_Timer.time_ms = 10;
			MCP_Wakeup_Timer.function_parameter = 4;
			SFT_start(&MCP_Wakeup_Timer);
			Init_worker_step++;
		break;
		
		case 3:
			//do nothing wile timer is ticking
		break;
		
		case 4: //NOT FINISH FOR UPLOAD !!!!!!!
			//send config
			SPI_Buffer[0] = SPI_READ_STATUS; 
			SPI_Sender.Direction = SPI_write;
			SPI_Sender.size = 3;
			MSQ_message_push(&MD_SPI_MSQ_queue, &SPI_Sender);
			Init_worker_step++;
			//send config data als array to can device
		break;
		
		case 5:
			//do nothing wile mcp2515 is getting data
		break;
		
		case 6:
			//init is ready
			MD_CAN_Inbound_Task.TaskStatus = IDLE;
			MD_CAN_Outbound_Task.TaskStatus = IDLE;
			Init_worker_step = 100;
		break;
		
			
	}	
}

/** If init is not goto next step.
Init is sending config data over spi to CAN interface. */
void next_init_step (void) {
	if (Init_worker_step < 100) {
		Init_worker_step++;
	}
}

 uint_fast8_t CAN_RxMOB_init(CAN_MOB *in_MOB) {
	 
 }
 
  uint_fast8_t CAN_send_Data(CAN_MOB *in_MOB) {
	  
  }
  
   /**
  * Moves CAN Messanges from SPI chip to MOB data Storage. 
  * Moves messages regardles if the are new. The check must separatly done. 
  * The function completly refills the MOB object. The only thin that will coresponde to the HW is the HW_Buffer everithing else will be rewritten during the function call
  * @param in_MOB Destination MOB
  * @return OK or ERROR
  */
 uint_fast8_t CAN_recive_Data(CAN_MOB *in_MOB) {
	 
 }
 
 uint_fast8_t CAN_check_new_Data (CAN_MOB *in_MOB) {
	 
 }
 
 uint_fast8_t CAN_check_mob_status (CAN_MOB *in_MOB)  {
	 
 }
 
 uint_fast8_t CAN_finish_mob (CAN_MOB *in_MOB) {
	 
 }
// #endif