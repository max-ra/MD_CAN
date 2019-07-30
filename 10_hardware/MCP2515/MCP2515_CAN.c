/*
 * MD_MCP2515.c
 *
 * Created: 29.07.2019 15:07:51
 */ 
// ##### Including #####
#include "MD_MCP2515.h"
#include "../../../02_kernel/scheduler/message_query.h"
#include "../../../02_kernel/scheduler/SoftwareTimer_SFT.h"
#include "../..MD_IO/MD_IO.h"

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
	MD_IO_SPI_CAN_Select_init();
	
	//Setup SPI link
	SPI_Sender.callback_SPI = &MCP2515_CAN_SPI_callback;
	SPI_Sender.DataPointer = &SPI_Buffer;
	SPI_Sender.Direction = SPI_write;
	SPI_Sender.size = 1;
	
	//Setup Timer
	MCP_Wakeup_Timer.function_parameter = 0;
	MCP_Wakeup_Timer.time_ms = 10;
	MCP_Wakeup_Timer.time_out_function = &MCP2515_CAN_wakeup;
}

void MCP2515_CAN_wakeup(uint_fast8_t temp) {
	//set stet at init
	Init_worker_step = temp;
}

void MCP2515_CAN_SPI_callback(uint_fast8_t status) {
	if (status == SPI_start) {
		MD_IO_SPI_CAN_Select_off();
	} else if (status == SPI_stop) {
		next_init_step();
		MD_IO_SPI_CAN_Select_on();
	}
}

void MD_CAN_Init_worker(void) {
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
		
		case 4:
			//send config
			SPI_Buffer[0] = ;
			SPI_Sender.Direction = SPI_write;
			SPI_Sender.size = 1;
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