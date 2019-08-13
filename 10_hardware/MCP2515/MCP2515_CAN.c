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
#include "../../../MD_SPI/10_hardware/MD_SPI_common.h"
#include <avr/delay.h>
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
 void mcp2515_write_register( uint8_t adress, uint8_t data );
void MCP2515_CAN_wakeup (uint_fast8_t temp);
void MCP2515_CAN_SPI_callback(uint_fast8_t status);
void next_init_step(void);
uint8_t mcp2515_read_register(uint8_t adress);


 uint8_t spi_putc( uint8_t data );
void CAN_init(void) {
	//setup Hardware
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
		MD_IO_SPI_CAN_Select_off();
	} else if (status == SPI_stop) {
		next_init_step();
		MD_IO_SPI_CAN_Select_on();
	}
}


void MD_CAN_Init_worker(void) {
	/*
	 // MCP2515 per Software Reset zuruecksetzten,
    // danach ist der MCP2515 im Configuration Mode
   MCP2515_CAN_SPI_callback(SPI_start);
    spi_putc( SPI_RESET );
    _delay_ms(10);
    MCP2515_CAN_SPI_callback(SPI_stop);
    
    // etwas warten bis sich der MCP2515 zurueckgesetzt hat
    _delay_ms(10);
    
    *//* 
     *  Einstellen des Bit Timings
     *  
     *  Fosc       = 16MHz
     *  BRP        = 7                (teilen durch 8)
     *  TQ = 2 * (BRP + 1) / Fosc  (=> 1 uS)
     *  
     *  Sync Seg   = 1TQ
     *  Prop Seg   = (PRSEG + 1) * TQ  = 1 TQ
     *  Phase Seg1 = (PHSEG1 + 1) * TQ = 3 TQ
     *  Phase Seg2 = (PHSEG2 + 1) * TQ = 3 TQ
     *  
     *  Bus speed  = 1 / (Total # of TQ) * TQ
     *             = 1 / 8 * TQ = 125 kHz
     */
    /*
    // BRP = 7
    mcp2515_write_register( CNF1, (1<<BRP0) );
    
    // Prop Seg und Phase Seg1 einstellen
    mcp2515_write_register( CNF2, (1<<BTLMODE)|(1<<PHSEG11)  );
    
    // Wake-up Filter deaktivieren, Phase Seg2 einstellen
    mcp2515_write_register( CNF3, (1<<PHSEG21) );
    
    
    
     // Einstellen der Filter

    
    // Buffer 0 : Empfangen aller Nachrichten
    mcp2515_write_register( RXB0CTRL, (1<<RXM1)|(1<<RXM0) );
    
    // Buffer 1 : Empfangen aller Nachrichten
    mcp2515_write_register( RXB1CTRL, (1<<RXM1)|(1<<RXM0) );
    
    // Alle Bits der Empfangsmaske loeschen, 
    // damit werden alle Nachrichten empfangen
    mcp2515_write_register( RXM0SIDH, 0 );
    mcp2515_write_register( RXM0SIDL, 0 );
    mcp2515_write_register( RXM0EID8, 0 );
    mcp2515_write_register( RXM0EID0, 0 );
    
    mcp2515_write_register( RXM1SIDH, 0 );
    mcp2515_write_register( RXM1SIDL, 0 );
    mcp2515_write_register( RXM1EID8, 0 );
    mcp2515_write_register( RXM1EID0, 0 );
    
    
     // Einstellen der Pin Funktionen
    
    
    // Deaktivieren der Pins RXnBF Pins (High Impedance State)
    mcp2515_write_register( BFPCTRL, 0 );
    
    // Device zurueck in den normalen Modus versetzten
    mcp2515_bit_modify( CANCTRL, 0xE0, 0);
	
	*/
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
			SPI_Buffer[0] = SPI_WRITE;
			SPI_Buffer[1] =  CNF1;
			SPI_Buffer[2] = 0x00;
			
			SPI_Sender.Direction = SPI_write;
			SPI_Sender.size = 3;
			MSQ_message_push(&MD_SPI_MSQ_queue, &SPI_Sender);
			Init_worker_step++;
			//send config data als array to can device
		break;
		
		case 5:
		//do nothing wile timer is ticking
		break;
		
		case 6:
			//send config
			SPI_Buffer[0] = SPI_WRITE;
			SPI_Buffer[1] =  CNF2;
			SPI_Buffer[2] = ((1<<BTLMODE) | (1<<PHSEG11));
			
			SPI_Sender.Direction = SPI_write;
			SPI_Sender.size = 3;
			MSQ_message_push(&MD_SPI_MSQ_queue, &SPI_Sender);
			Init_worker_step++;
			//send config data als array to can device
		break;
		
		case 7:
		//do nothing wile timer is ticking
		break;
		
		case 8:
			//send config
			SPI_Buffer[0] = SPI_WRITE;
			SPI_Buffer[1] =  CNF3;
			SPI_Buffer[2] = (1<<PHSEG21);
			
			SPI_Sender.Direction = SPI_write;
			SPI_Sender.size = 3;
			MSQ_message_push(&MD_SPI_MSQ_queue, &SPI_Sender);
			Init_worker_step++;
			//send config data als array to can device
		break;		
		
				case 9:
				//do nothing wile timer is ticking
				break;
		
		case 10:
			//send config
			SPI_Buffer[0] = SPI_WRITE;
			SPI_Buffer[1] = RXB0CTRL;
			SPI_Buffer[2] = ((1<<RXM1) | (1<<RXM0));
			
			SPI_Sender.Direction = SPI_write;
			SPI_Sender.size = 3;
			MSQ_message_push(&MD_SPI_MSQ_queue, &SPI_Sender);
			Init_worker_step++;
			//send config data als array to can device
		break;	
		
		case 11:
		//do nothing wile timer is ticking
		break;
		case 12:
			//send config
			SPI_Buffer[0] = SPI_WRITE;
			SPI_Buffer[1] = RXB1CTRL;
			SPI_Buffer[2] = ((1<<RXM1) | (1<<RXM0));
			
			SPI_Sender.Direction = SPI_write;
			SPI_Sender.size = 3;
			MSQ_message_push(&MD_SPI_MSQ_queue, &SPI_Sender);
			Init_worker_step++;
			//send config data als array to can device
		break;	
		case 13:
		//do nothing wile timer is ticking
		break;		
		case 14:
			//send config
			SPI_Buffer[0] = SPI_WRITE;
			SPI_Buffer[1] = RXM0SIDH;
			SPI_Buffer[2] = 0;
			
			SPI_Sender.Direction = SPI_write;
			SPI_Sender.size = 3;
			MSQ_message_push(&MD_SPI_MSQ_queue, &SPI_Sender);
			Init_worker_step++;
			//send config data als array to can device
		break;	
		case 15:
		//do nothing wile timer is ticking
		break;		
		case 16:
			//send config
			SPI_Buffer[0] = SPI_WRITE;
			SPI_Buffer[1] = RXM0SIDH;
			SPI_Buffer[2] = 0;
			
			SPI_Sender.Direction = SPI_write;
			SPI_Sender.size = 3;
			MSQ_message_push(&MD_SPI_MSQ_queue, &SPI_Sender);
			Init_worker_step++;
			//send config data als array to can device
		break;	
				case 17:
				//do nothing wile timer is ticking
				break;
		case 18:
			//send config
			SPI_Buffer[0] = SPI_WRITE;
			SPI_Buffer[1] = RXM0SIDL;
			SPI_Buffer[2] = 0;
			
			SPI_Sender.Direction = SPI_write;
			SPI_Sender.size = 3;
			MSQ_message_push(&MD_SPI_MSQ_queue, &SPI_Sender);
			Init_worker_step++;
			//send config data als array to can device
		break;	
			case 19:
			//do nothing wile timer is ticking
			break;	
		case 20:
			//send config
			SPI_Buffer[0] = SPI_WRITE;
			SPI_Buffer[1] = RXM0EID8;
			SPI_Buffer[2] = 0;
			
			SPI_Sender.Direction = SPI_write;
			SPI_Sender.size = 3;
			MSQ_message_push(&MD_SPI_MSQ_queue, &SPI_Sender);
			Init_worker_step++;
			//send config data als array to can device
		break;	
		case 21:
		//do nothing wile timer is ticking
		break;
		case 22:
			//send config
			SPI_Buffer[0] = SPI_WRITE;
			SPI_Buffer[1] = RXM0EID0;
			SPI_Buffer[2] = 0;
			
			SPI_Sender.Direction = SPI_write;
			SPI_Sender.size = 3;
			MSQ_message_push(&MD_SPI_MSQ_queue, &SPI_Sender);
			Init_worker_step++;
			//send config data als array to can device
		break;	
			case 23:
			//do nothing wile timer is ticking
			break;	
		case 24:
			//send config
			SPI_Buffer[0] = SPI_WRITE;
			SPI_Buffer[1] = RXM1SIDH;
			SPI_Buffer[2] = 0;
			
			SPI_Sender.Direction = SPI_write;
			SPI_Sender.size = 3;
			MSQ_message_push(&MD_SPI_MSQ_queue, &SPI_Sender);
			Init_worker_step++;
			//send config data als array to can device
		break;			
		case 25:
		//do nothing wile timer is ticking
		break;
		case 26:
			//send config
			SPI_Buffer[0] = SPI_WRITE;
			SPI_Buffer[1] = RXM1SIDL;
			SPI_Buffer[2] = 0;
			
			SPI_Sender.Direction = SPI_write;
			SPI_Sender.size = 3;
			MSQ_message_push(&MD_SPI_MSQ_queue, &SPI_Sender);
			Init_worker_step++;
			//send config data als array to can device
		break;	
			case 27:
			//do nothing wile timer is ticking
			break;	
		case 28:
			//send config
			SPI_Buffer[0] = SPI_WRITE;
			SPI_Buffer[1] = RXM1EID8;
			SPI_Buffer[2] = 0;
			
			SPI_Sender.Direction = SPI_write;
			SPI_Sender.size = 3;
			MSQ_message_push(&MD_SPI_MSQ_queue, &SPI_Sender);
			Init_worker_step++;
			//send config data als array to can device
		break;	
		case 29:
		//do nothing wile timer is ticking
		break;		
		case 30:
			//send config
			SPI_Buffer[0] = SPI_WRITE;
			SPI_Buffer[1] = RXM1EID0;
			SPI_Buffer[2] = 0;
			
			SPI_Sender.Direction = SPI_write;
			SPI_Sender.size = 3;
			MSQ_message_push(&MD_SPI_MSQ_queue, &SPI_Sender);
			Init_worker_step++;
			//send config data als array to can device
		break;	
		case 31:
		//do nothing wile timer is ticking
		break;		
		case 32:
			//send config
			SPI_Buffer[0] = SPI_WRITE;
			SPI_Buffer[1] = BFPCTRL;
			SPI_Buffer[2] = 0;
			
			SPI_Sender.Direction = SPI_write;
			SPI_Sender.size = 3;
			MSQ_message_push(&MD_SPI_MSQ_queue, &SPI_Sender);
			Init_worker_step++;
			//send config data als array to can device
		break;	
			case 33:
			//do nothing wile timer is ticking
			break;
		case 34:
			//send config
			SPI_Buffer[0] = SPI_BIT_MODIFY;
			SPI_Buffer[1] = CANCTRL;
			SPI_Buffer[2] = 0xE0;
			SPI_Buffer[3] = 0;
			
			SPI_Sender.Direction = SPI_write;
			SPI_Sender.size = 4;
			MSQ_message_push(&MD_SPI_MSQ_queue, &SPI_Sender);
			Init_worker_step++;
			//send config data als array to can device
		break;	

		case 35:
			//do nothing wile mcp2515 is getting data
		break;
		
		case 36:
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
	  /*		SPI_Buffer[0] = SPI_WRITE;
			SPI_Buffer[1] = TXB0SIDH;
			SPI_Buffer[2] = (uint8_t)((in_MOB->Identifyer.standard) >> 3);
			SPI_Buffer[3] = SPI_WRITE;
			SPI_Buffer[4] = TXB0SIDL;
			SPI_Buffer[5] = (uint8_t)(in_MOB->Identifyer.standard <<5);
			
			
			    // Ist die Nachricht ein "Remote Transmit Request"
			if (in_MOB->Command == CMD_REPLY)  {
        // Eine RTR Nachricht hat zwar eine Laenge, 
        //   aber keine Daten 
        
        // Nachrichten Laenge + RTR einstellen
			SPI_Buffer[6] = SPI_write;
			SPI_Buffer[7] = TXB0DLC;
			SPI_Buffer[8] = ((1<<RTR) | in_MOB->data_length);

		 } else {
        // Nachrichten Laenge einstellen
			SPI_Buffer[9] = SPI_write;
			SPI_Buffer[10] = TXB0DLC;
			SPI_Buffer[11] = in_MOB->data_length;
        
		 // Daten
        
			SPI_Buffer[12] = SPI_write;
			SPI_Buffer[13] = TXB0D0 ;
			SPI_Buffer[14] = (in_MOB->data[0]);
			
			SPI_Buffer[15] = SPI_write;
			SPI_Buffer[16] = TXB0D0 + 1 ;
			SPI_Buffer[17] = (in_MOB->data[1]);		

			SPI_Buffer[18] = SPI_write;
			SPI_Buffer[19] = TXB0D0 + 2;
			SPI_Buffer[20] = (in_MOB->data[2]);
			
			SPI_Buffer[21] = SPI_write;
			SPI_Buffer[22] = TXB0D0 + 3;
			SPI_Buffer[23] = (in_MOB->data[3]);
			
			SPI_Buffer[24] = SPI_write;
			SPI_Buffer[25] = TXB0D0 + 4;
			SPI_Buffer[26] = (in_MOB->data[4]);
			
			SPI_Buffer[27] = SPI_write;
			SPI_Buffer[28] = TXB0D0 + 5;
			SPI_Buffer[29] = (in_MOB->data[5]);
			
			SPI_Buffer[30] = SPI_write;
			SPI_Buffer[31] = TXB0D0 + 6;
			SPI_Buffer[32] = (in_MOB->data[6]);
			
			SPI_Buffer[33] = SPI_write;
			SPI_Buffer[34] = TXB0D0 + 7;
			SPI_Buffer[35] = (in_MOB->data[7]);		    
    }
			SPI_Buffer[36] = (SPI_RTS | 0x01);
    
			SPI_Sender.Direction = SPI_write;
			SPI_Sender.size = 37;
			MSQ_message_push(&MD_SPI_MSQ_queue, &SPI_Sender); */
	  
	  mcp2515_read_register(CNF1);
	  
	  
	   uint8_t length = in_MOB->data_length;
    
    // ID einstellen
	uint8_t HIGH;
	HIGH = (uint8_t)(in_MOB->Identifyer.standard >> 3);
    mcp2515_write_register(TXB0SIDH, (uint8_t) (in_MOB->Identifyer.standard >> 3));
    mcp2515_write_register(TXB0SIDL, (uint8_t) (in_MOB->Identifyer.standard <<5));
    
    // Ist die Nachricht ein "Remote Transmit Request"
    if (in_MOB->Command == CMD_REPLY)
    {
        /* Eine RTR Nachricht hat zwar eine Laenge, 
           aber keine Daten */
        
        // Nachrichten Laenge + RTR einstellen
        mcp2515_write_register(TXB0DLC, (1<<RTR) | length);
    }
    else
    {
        // Nachrichten Laenge einstellen
        mcp2515_write_register(TXB0DLC, length);
        
        // Daten
        for (uint8_t i=0;i<length;i++) {
            mcp2515_write_register(TXB0D0 + i, in_MOB->data[i]);
        }
    }
    
    // CAN Nachricht verschicken
		MCP2515_CAN_SPI_callback(SPI_start);
		spi_putc(SPI_RTS | 0x01);
		MCP2515_CAN_SPI_callback(SPI_stop);
// 			SPI_Buffer[0] = (SPI_RTS | 0x01);
// 	
// 			
// 			SPI_Sender.Direction = SPI_write;
// 			SPI_Sender.size = 1;
// 			MSQ_message_push(&MD_SPI_MSQ_queue, &SPI_Sender);
return OK;
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
 
 //check if new data is availabel 
 //todo: onyl qick hackarount for FSG
 uint_fast8_t CAN_check_new_Data (CAN_MOB *in_MOB) {
	 //if (in_MOB->Hardware_buffer == 1) {
		 
 }
 
 uint_fast8_t CAN_check_mob_status (CAN_MOB *in_MOB)  {
	 
 }
 
 uint_fast8_t CAN_finish_mob (CAN_MOB *in_MOB) {
	 
 }
 
 
 void mcp2515_write_register( uint8_t adress, uint8_t data )
 {
	 // /CS des MCP2515 auf Low ziehen
	 MCP2515_CAN_SPI_callback(SPI_start);

	 spi_putc(SPI_WRITE);
	 spi_putc(adress);
	 spi_putc(data);
	 
	 // /CS Leitung wieder freigeben
	 MCP2515_CAN_SPI_callback(SPI_stop);
 }
 
 
 uint8_t spi_putc( uint8_t data )
 {
	 // Sendet ein Byte
	 SPDR0 = data;
	 
	 // Wartet bis Byte gesendet wurde
	 while( !( SPSR0 & (1<<SPIF) ) )
	 ;
	 
	 return SPDR0;
 }
 
 void mcp2515_bit_modify(uint8_t adress, uint8_t mask, uint8_t data)
{
	// /CS des MCP2515 auf Low ziehen
MCP2515_CAN_SPI_callback(SPI_start);
	
	spi_putc(SPI_BIT_MODIFY);
	spi_putc(adress);
	spi_putc(mask);
	spi_putc(data);
	
	// /CS Leitung wieder freigeben
MCP2515_CAN_SPI_callback(SPI_stop);
}

uint8_t mcp2515_read_register(uint8_t adress)
{
	uint8_t data;
	
	// /CS des MCP2515 auf Low ziehen
	MCP2515_CAN_SPI_callback(SPI_start);
	spi_putc(SPI_READ);
	spi_putc(adress);
	
	data = spi_putc(0xff);
	
	// /CS Leitung wieder freigeben
	MCP2515_CAN_SPI_callback(SPI_stop);
	
	return data;
}
 
// #endif

