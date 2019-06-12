// ##### Including #####
#include "PIC33EP_CAN.h"

// ##### Settings #####


// ##### definition #####
	

// ##### Status definition #####
	

// ##### variable #####

/* Define ECAN Message Buffers */
__eds__ ECAN1MSGBUF ecan1msgBuf __attribute__((eds,aligned(ECAN1_MSG_BUF_LENGTH*16)));
uint_fast16_t Buffer_State; 


// ##### function #####
void clearRxFlags(unsigned char buffer_number);

void CAN_init(void) {
// Clear Buffer usage
    Buffer_State = 0;
    
//Init Connection to board
    //Ennable CAN Tranciver
     //ANSELBbits.ANSB = 0;              //disable Analoug function
     TRISBbits.TRISB10 = 0;              //Set as output pin
     LATBbits.LATB10 = 0;                //switch to ground (enable the tranciver low active)
     
    //Input Setup     
     //ANSELBbits.ANSB3 = 0;              //Disabel Analouge function at B3
     TRISFbits.TRISF1 = 1;              //Set as input pin
     RPINR26bits.C1RXR = 0b1100001;     //CAN1RX to RP97 

    //Output Setup
     //ANSELCbits.ANSC9    = 0;           //Disable analouge functions
     TRISCbits.TRISC9    = 0;           //TX as Output
     RPOR7bits.RP57R     = 0b001110;    //CAN1TX to RC9
     
//Switch CAN Interface to Setup mode
	C1CTRL1bits.REQOP=4;
	while(C1CTRL1bits.OPMODE != 4);
    
//Setup Timings and Frequency
    //Set CAN settings
        C1CTRL1bits.CSIDL   = 0;     // continue in idle mode
        C1CTRL1bits.CANCKS  = 0;     // Fcan = Fp S.297 Handbuch
        
    //CAN Baut Rate configuration
        C1CFG1bits.BRP      = 1;        //Baud Rate Prescaler Tq = 4/Fcan S.304
        C1CFG1bits.SJW      = 0b10;     //Synchronization Jump Width (SJW) = 3 x TQ S.304
        C1CFG2bits.PRSEG    = 4;        //Propagation Time Segment: Delay = 5TQ S.305
        C1CFG2bits.SEG1PH   = 4;        //Phase Segment 1: Phase1 = 5TQ S.305
        C1CFG2bits.SEG2PHTS = 1;        //Phase Segment 2 Time select: Freely programmable S.305
        C1CFG2bits.SEG2PH   = 4;        //Phase Segment 2: Phase2 = 5TQ S.305
        C1CFG2bits.SAM      = 1;        //Sample of the CAN Bus Line: Bus line is sampled three times at the sample point S.305
        //Übertragungsrate siehe ECAN S59 Reference Manual FBaud = 16MHz/16TQ = 1MBaud
    
//Buffer 
    C1FCTRLbits.DMABS = 0b100;       //16Buffers in DMA RAM
    //C1FCTRLbits.FSA = 16;            //FIFO starts at Buffer 17
            
//DMA Setup
    CAN_init_DMA();
    
//Switch configuration windo to Filter
    C1CTRL1bits.WIN=0b1;
    
//Accepetence mask mapping (3 Mask are available)
    C1FMSKSEL1bits.F0MSK = 0b00;       //Set Acceptance Mask 0 to Filter 0 
    C1FMSKSEL1bits.F1MSK = 0b00;       //Set Acceptance Mask 0 to Filter 1
    C1FMSKSEL1bits.F2MSK = 0b00;       //Set Acceptance Mask 0 to Filter 2
    C1FMSKSEL1bits.F3MSK = 0b00;       //Set Acceptance Mask 0 to Filter 3
    C1FMSKSEL1bits.F4MSK = 0b00;       //Set Acceptance Mask 0 to Filter 4
    C1FMSKSEL1bits.F5MSK = 0b00;       //Set Acceptance Mask 0 to Filter 5
    C1FMSKSEL1bits.F6MSK = 0b00;       //Set Acceptance Mask 0 to Filter 6
    C1FMSKSEL1bits.F7MSK = 0b00;       //Set Acceptance Mask 0 to Filter 7 
    
    C1FMSKSEL2bits.F8MSK = 0b00;       //Set Acceptance Mask 0 to Filter 8 
    C1FMSKSEL2bits.F9MSK = 0b00;       //Set Acceptance Mask 0 to Filter 9
    C1FMSKSEL2bits.F10MSK = 0b00;       //Set Acceptance Mask 0 to Filter 10
    C1FMSKSEL2bits.F11MSK = 0b00;       //Set Acceptance Mask 0 to Filter 11
    C1FMSKSEL2bits.F12MSK = 0b00;       //Set Acceptance Mask 0 to Filter 12
    C1FMSKSEL2bits.F13MSK = 0b00;       //Set Acceptance Mask 0 to Filter 13
    C1FMSKSEL2bits.F14MSK = 0b00;       //Set Acceptance Mask 0 to Filter 14
    C1FMSKSEL2bits.F15MSK = 0b00;       //Set Acceptance Mask 0 to Filter 15    

//Acceptance mask setup
   C1RXM0SIDbits.SID = 0x7FF; //CAN_FILTERMASK2REG_SID(0x7FF);  //Check every bit of ther standrd message id
   C1RXM0SIDbits.MIDE = 0;
    
//Disable acces to filter config
   C1CTRL1bits.WIN=0;    
       
//Set CAN Controller in normal mode
   C1CTRL1bits.REQOP=0;
   while(C1CTRL1bits.OPMODE != 0);	
   
//Clear Buffer
  C1RXFUL1=C1RXFUL2=C1RXOVF1=C1RXOVF2=0x0000;
  
//Change Buffer behavior 
	/* ECAN1, Buffer 0 is a Transmit Buffer */
	C1TR01CONbits.TXEN0=1;			
	/* ECAN1, Buffer 1 is a Receive Buffer */
	C1TR01CONbits.TXEN1=0;	
	/* ECAN1, Buffer 2 is a Receive Buffer */
	C1TR23CONbits.TXEN2=0;	
	/* ECAN1, Buffer 3 is a Receive Buffer */
	C1TR23CONbits.TXEN3=0;	
    /* ECAN1, Buffer 4 is a Receive Buffer */
	C1TR45CONbits.TXEN4=0;
    /* ECAN1, Buffer 5 is a Receive Buffer */
	C1TR45CONbits.TXEN5=0;
    /* ECAN1, Buffer 6 is a Receive Buffer */
	C1TR67CONbits.TXEN6=0;
    /* ECAN1, Buffer 7 is a Receive Buffer */
	C1TR67CONbits.TXEN7=0;
    /* ECAN1, Buffer 8 - 15 are only Receive Buffers */

	
    
	/* Message Buffer 0 Priority Level */
	C1TR01CONbits.TX0PRI=0b11; 		
    
//Set hardware buffer 0 to occupied (Transmit Buffer)
    Buffer_State |= (1<<0);
    
//Interupt setup
    /* configure the device to interrupt on the receive buffer full flag */
	/* clear the buffer full flags */
	C1RXFUL1=0;
	C1INTFbits.RBIF=0;
   
}

/******************************************************************************
*                                                                             
*    Function:			initDMAECAN
*    Description:       Initialises the DMA to be used with ECAN module                                                        
*                       Channel 0 of the DMA is configured to Tx ECAN messages
* 						of ECAN module 1. 
*						Channel 2 is uconfigured to Rx ECAN messages of module 1.                                                      
*    Arguments:			
*	 Author:            Jatinder Gharoo                                                      
*	                                                                 
*                                                                              
******************************************************************************/
void CAN_init_DMA (void) {
 	/* initialise the DMA channel 0 for ECAN Tx */
	/* clear the collission flags */
	//DMACS0=0;	
    /* setup channel 0 for peripheral indirect addressing mode 
    normal operation, word operation and select as Tx to peripheral */
    DMA0CON=0x2020; 
    /* setup the address of the peripheral ECAN1 (C1TXD) */ 
	DMA0PAD= (volatile unsigned int)&C1TXD; //0x0442;
	/* Set the data block transfer size of 8 */
 	DMA0CNT=7;
 	/* automatic DMA Tx initiation by DMA request CAN1 TX Data Request*/
	DMA0REQ=0x0046;	
	/* DPSRAM atart adddress offset value */ 
	DMA0STAL = __builtin_dmaoffset(&ecan1msgBuf);
    DMA0STAH = 0x0000; 	
	/* enable the channel */
	DMA0CONbits.CHEN=1;
	
	/* initialise the DMA channel 2 for ECAN Rx */
	/* clear the collission flags */
	//DMACS0=0;
    /* setup channel 2 for peripheral indirect addressing mode 
    normal operation, word operation and select as Rx to peripheral */
    DMA2CON=0x0020;
    /* setup the address of the peripheral ECAN1 (C1RXD) */ 
	DMA2PAD=(volatile unsigned int)&C1RXD; //0x0440;	
 	/* Set the data block transfer size of 8 */
 	DMA2CNT=7;
 	/* automatic DMA Rx initiation by DMA request */
	DMA2REQ=0x0022;	
	/* DPSRAM atart adddress offset value */ 
	DMA2STAL =__builtin_dmaoffset(&ecan1msgBuf);	
    DMA2STAH = 0x0000;
	/* enable the channel */
	DMA2CONbits.CHEN=1;   
}

 uint_fast8_t CAN_RxMOB_init(CAN_MOB *in_MOB) {
//Check if Hardware Buffer is allready in use
     if (Buffer_State & (1<<(in_MOB->Hardware_buffer))) {
         return Error;
     }
     
     
//Switch CAN Interface to Setup mode
	C1CTRL1bits.REQOP=4;
	while(C1CTRL1bits.OPMODE != 4);
     
//Switch configuration windo to Filter
    C1CTRL1bits.WIN=0b1;
     
    
if (in_MOB->frame_type==standard) {
    switch(in_MOB->Hardware_buffer) {
        case 1:
           //Set filter id
            C1RXF0SIDbits.SID = in_MOB->Identifyer.standard;
    
           /* set filter to accept standard id only */
            C1RXF0SIDbits.EXIDE = 0;       
    
           /* acceptance filter 0 to use buffer 1 for incoming messages */
            C1BUFPNT1bits.F0BP=1;
    
           /* enable filter 0 */
            C1FEN1bits.FLTEN0=1;
        break;    
        
        case 2:
           //Set filter id
            C1RXF1SIDbits.SID = in_MOB->Identifyer.standard;
    
           /* set filter to check for standard ID and accept standard id only */
            C1RXF1SIDbits.EXIDE = 0;      
    
           /* acceptance filter 1 to use buffer 2 for incoming messages */
            C1BUFPNT1bits.F1BP=2;
    
           /* enable filter 1 */
            C1FEN1bits.FLTEN1=1;
        break;    
        
        case 3:
           //Set filter id
            C1RXF2SIDbits.SID = in_MOB->Identifyer.standard;
    
           /* set filter to check for standard ID and accept standard id only */
            C1RXF2SIDbits.EXIDE = 0;      
    
           /* acceptance filter to use buffer 3 for incoming messages */
            C1BUFPNT1bits.F2BP=3;
    
           /* enable filter 1 */
            C1FEN1bits.FLTEN2=1;
        break;    
        
        case 4:
           //Set filter id
            C1RXF3SIDbits.SID = in_MOB->Identifyer.standard;
    
           /* set filter to check for standard ID and accept standard id only */
            C1RXF3SIDbits.EXIDE = 0;      
    
           /* acceptance filter to use buffer 3 for incoming messages */
            C1BUFPNT1bits.F3BP=4;
    
           /* enable filter 1 */
            C1FEN1bits.FLTEN3=1;
        break;  
        
        case 5:
           //Set filter id
            C1RXF4SIDbits.SID = in_MOB->Identifyer.standard;
    
           /* set filter to check for standard ID and accept standard id only */
            C1RXF4SIDbits.EXIDE = 0;      
    
           /* acceptance filter to use buffer 3 for incoming messages */
            C1BUFPNT2bits.F4BP=5;
    
           /* enable filter 1 */
            C1FEN1bits.FLTEN4=1;
        break;  
        
        case 6:
           //Set filter id
            C1RXF5SIDbits.SID = in_MOB->Identifyer.standard;
    
           /* set filter to check for standard ID and accept standard id only */
            C1RXF5SIDbits.EXIDE = 0;      
    
           /* acceptance filter to use buffer 3 for incoming messages */
            C1BUFPNT2bits.F5BP=6;
    
           /* enable filter 1 */
            C1FEN1bits.FLTEN5=1;
        break;  
        
        case 7:
           //Set filter id
            C1RXF6SIDbits.SID = in_MOB->Identifyer.standard;
    
           /* set filter to check for standard ID and accept standard id only */
            C1RXF6SIDbits.EXIDE = 0;      
    
           /* acceptance filter to use buffer 3 for incoming messages */
            C1BUFPNT2bits.F6BP=7;
    
           /* enable filter 1 */
            C1FEN1bits.FLTEN6=1;
        break;  
        
        case 8:
           //Set filter id
            C1RXF7SIDbits.SID = in_MOB->Identifyer.standard;
    
           /* set filter to check for standard ID and accept standard id only */
            C1RXF7SIDbits.EXIDE = 0;      
    
           /* acceptance filter to use buffer 3 for incoming messages */
            C1BUFPNT2bits.F7BP=8;
    
           /* enable filter 1 */
            C1FEN1bits.FLTEN7=1;
        break;  
        
        case 9:
           //Set filter id
            C1RXF8SIDbits.SID = in_MOB->Identifyer.standard;
    
           /* set filter to check for standard ID and accept standard id only */
            C1RXF8SIDbits.EXIDE = 0;      
    
           /* acceptance filter to use buffer 3 for incoming messages */
            C1BUFPNT3bits.F8BP=9;
    
           /* enable filter 1 */
            C1FEN1bits.FLTEN8=1;
        break;  
        
        case 10:
           //Set filter id
            C1RXF9SIDbits.SID = in_MOB->Identifyer.standard;
    
           /* set filter to check for standard ID and accept standard id only */
            C1RXF9SIDbits.EXIDE = 0;      
    
           /* acceptance filter to use buffer 3 for incoming messages */
            C1BUFPNT3bits.F9BP=10;
    
           /* enable filter 1 */
            C1FEN1bits.FLTEN9=1;
        break;  
        
        case 11:
           //Set filter id
            C1RXF10SIDbits.SID = in_MOB->Identifyer.standard;
    
           /* set filter to check for standard ID and accept standard id only */
            C1RXF10SIDbits.EXIDE = 0;      
    
           /* acceptance filter to use buffer 3 for incoming messages */
            C1BUFPNT3bits.F10BP=11;
    
           /* enable filter 1 */
            C1FEN1bits.FLTEN10=1;
        break;  
        
        case 12:
           //Set filter id
            C1RXF11SIDbits.SID = in_MOB->Identifyer.standard;
    
           /* set filter to check for standard ID and accept standard id only */
            C1RXF11SIDbits.EXIDE = 0;      
    
           /* acceptance filter to use buffer 3 for incoming messages */
            C1BUFPNT3bits.F11BP=12;
    
           /* enable filter 1 */
            C1FEN1bits.FLTEN11=1;
        break; 
        
        case 13:
           //Set filter id
            C1RXF12SIDbits.SID = in_MOB->Identifyer.standard;
    
           /* set filter to check for standard ID and accept standard id only */
            C1RXF12SIDbits.EXIDE = 0;      
    
           /* acceptance filter to use buffer 3 for incoming messages */
            C1BUFPNT4bits.F12BP=13;
    
           /* enable filter 1 */
            C1FEN1bits.FLTEN12=1;
        break; 
        
        case 14:
           //Set filter id
            C1RXF13SIDbits.SID = in_MOB->Identifyer.standard;
    
           /* set filter to check for standard ID and accept standard id only */
            C1RXF13SIDbits.EXIDE = 0;      
    
           /* acceptance filter to use buffer 3 for incoming messages */
            C1BUFPNT4bits.F13BP=14;
    
           /* enable filter 1 */
            C1FEN1bits.FLTEN13=1;
        break; 

        case 15:
           //Set filter id
            C1RXF14SIDbits.SID = in_MOB->Identifyer.standard;
    
           /* set filter to check for standard ID and accept standard id only */
            C1RXF14SIDbits.EXIDE = 0;      
    
           /* acceptance filter to use buffer 3 for incoming messages */
            C1BUFPNT4bits.F14BP=15;
    
           /* enable filter 1 */
            C1FEN1bits.FLTEN14=1;
        break; 
        
        default:
            return Error;
        break;
    }
   
}    
    
// set hw buffer als occupied
Buffer_State |= (1<<in_MOB->Hardware_buffer);
    
//Disable acces to filter config
C1CTRL1bits.WIN=0;
   
//Set CAN Controller in normal mode
C1CTRL1bits.REQOP=0;
while(C1CTRL1bits.OPMODE != 0);	
   
//Clear Buffer
C1RXFUL1=C1RXFUL2=C1RXOVF1=C1RXOVF2=0x0000;

return OK;
 }
 
 uint_fast8_t CAN_send_Data(CAN_MOB *in_MOB) {
    uint_fast16_t word0=0;
	uint_fast16_t word1=0;
	uint_fast16_t word2=0;
	
    //Check if Send Buffer 0 is Selected
    if (!(in_MOB->Hardware_buffer==0)) {
        return Error;
    }
    
	/*
	Message Format: 
	Word0 : 0bUUUx xxxx xxxx xxxx
			     |____________|||
 					SID10:0   SRR IDE(bit 0)     
	Word1 : 0bUUUU xxxx xxxx xxxx
			   	   |____________|
						EID17:6
	Word2 : 0bxxxx xxx0 UUU0 xxxx
			  |_____||	     |__|
			  EID5:0 RTR   	  DLC
	
	Remote Transmission Request Bit for standard frames 
	SRR->	"0"	 Normal Message 
			"1"  Message will request remote transmission
	Substitute Remote Request Bit for extended frames 
	SRR->	should always be set to "1" as per CAN specification
	
	Extended  Identifier Bit			
	IDE-> 	"0"  Message will transmit standard identifier
	   		"1"  Message will transmit extended identifier
	
	Remote Transmission Request Bit for extended frames 
	RTR-> 	"0"  Message transmitted is a normal message
			"1"  Message transmitted is a remote message
	Don't care for standard frames 
	*/
		
	/* check to see if the message has an extended ID */
	if(in_MOB->frame_type == extendet)
	{
		/* get the extended message id EID28..18*/		
		word0=(in_MOB->Identifyer.extendet & 0x1FFC0000) >> 16;			
		/* set the SRR and IDE bit */
		word0=word0+0x0003;
		/* the the value of EID17..6 */
		word1=(in_MOB->Identifyer.extendet & 0x0003FFC0) >> 6;
		/* get the value of EID5..0 for word 2 */
		word2=(in_MOB->Identifyer.extendet & 0x0000003F) << 10;			
	}	
	else
	{
		/* get the SID */
		word0=((in_MOB->Identifyer.standard & 0x000007FF) << 2);	
	}
	/* check to see if the message is an RTR message */
	if((in_MOB->Command==CMD_RX_REMOTE) || (in_MOB->Command==CMD_TX_REMOTE)) {		
		if(in_MOB->frame_type==extendet) {
			word2=word2 | 0x0200;
		} else {
			word0=word0 | 0x0002;	
		}
        
        //Muss nicht die Nachrichten immer Im buffer 0 Abgelegt werden. Dieser ist als Sendebuffer konfiguriert??
		ecan1msgBuf[in_MOB->Hardware_buffer][0]=word0;
		ecan1msgBuf[in_MOB->Hardware_buffer][1]=word1;
		ecan1msgBuf[in_MOB->Hardware_buffer][2]=word2;
	} else {
		word2=word2+(in_MOB->data_length & 0x0F);
		ecan1msgBuf[in_MOB->Hardware_buffer][0]=word0;
		ecan1msgBuf[in_MOB->Hardware_buffer][1]=word1;
		ecan1msgBuf[in_MOB->Hardware_buffer][2]=word2;
		/* fill the data */
		ecan1msgBuf[in_MOB->Hardware_buffer][3]=((in_MOB->data[1] << 8) + in_MOB->data[0]);
		ecan1msgBuf[in_MOB->Hardware_buffer][4]=((in_MOB->data[3] << 8) + in_MOB->data[2]);
		ecan1msgBuf[in_MOB->Hardware_buffer][5]=((in_MOB->data[5] << 8) + in_MOB->data[4]);
		ecan1msgBuf[in_MOB->Hardware_buffer][6]=((in_MOB->data[7] << 8) + in_MOB->data[6]);
	}
	/* set the message for transmission */
	C1TR01CONbits.TXREQ0=1;
    
    return OK;
 }

 
 /**
  * Moves CAN Messanges from DMA Ram to MOB data Storage. 
  * Moves messages regardles if the are new. The check must separatly done. 
  * The function completly refills the MOB object. The only thin that will coresponde to the HW is the HW_Buffer everithing else will be rewritten during the function call
  * @param in_MOB Destination MOB
  * @return OK or ERROR
  */
 uint_fast8_t CAN_recive_Data(CAN_MOB *in_MOB) {
    unsigned int ide=0;
	unsigned int rtr=0;
	unsigned long id=0;
			
	/*
	Standard Message Format: 
	Word0 : 0bUUUx xxxx xxxx xxxx
			     |____________|||
 					SID10:0   SRR IDE(bit 0)     
	Word1 : 0bUUUU xxxx xxxx xxxx
			   	   |____________|
						EID17:6
	Word2 : 0bxxxx xxx0 UUU0 xxxx
			  |_____||	     |__|
			  EID5:0 RTR   	  DLC
	word3-word6: data bytes
	word7: filter hit code bits
	
	Remote Transmission Request Bit for standard frames 
	SRR->	"0"	 Normal Message 
			"1"  Message will request remote transmission
	Substitute Remote Request Bit for extended frames 
	SRR->	should always be set to "1" as per CAN specification
	
	Extended  Identifier Bit			
	IDE-> 	"0"  Message will transmit standard identifier
	   		"1"  Message will transmit extended identifier
	
	Remote Transmission Request Bit for extended frames 
	RTR-> 	"0"  Message transmitted is a normal message
			"1"  Message transmitted is a remote message
	Don't care for standard frames 
	*/
		
	/* read word 0 to see the message type */
	ide=ecan1msgBuf[in_MOB->Hardware_buffer][0] & 0x0001;			
	
	/* check to see what type of message it is */
	/* message is standard identifier */
	if(ide==0)
	{
		in_MOB->Identifyer.standard=(ecan1msgBuf[in_MOB->Hardware_buffer][0] & 0x1FFC) >> 2;		
		in_MOB->frame_type=standard;
		rtr=ecan1msgBuf[in_MOB->Hardware_buffer][0] & 0x0002;
	}
	/* mesage is extended identifier */
	else
	{
		id=ecan1msgBuf[in_MOB->Hardware_buffer][0] & 0x1FFC;		
		in_MOB->Identifyer.extendet=id << 16;
		id=ecan1msgBuf[in_MOB->Hardware_buffer][1] & 0x0FFF;
		in_MOB->Identifyer.extendet=in_MOB->Identifyer.extendet+(id << 6);
		id=(ecan1msgBuf[in_MOB->Hardware_buffer][2] & 0xFC00) >> 10;
		in_MOB->Identifyer.extendet =  in_MOB->Identifyer.extendet + id;		
		in_MOB->frame_type=extendet;
		rtr=ecan1msgBuf[in_MOB->Hardware_buffer][2] & 0x0200;
	}
	/* check to see what type of message it is */
	/* RTR message */
	if(rtr==1)
	{
		in_MOB->Command=CMD_RX_REMOTE;	
	}
	/* normal message */
	else
	{
		in_MOB->Command=CMD_RX_DATA;
		in_MOB->data[0]=(unsigned char)ecan1msgBuf[in_MOB->Hardware_buffer][3];
		in_MOB->data[1]=(unsigned char)((ecan1msgBuf[in_MOB->Hardware_buffer][3] & 0xFF00) >> 8);
		in_MOB->data[2]=(unsigned char)ecan1msgBuf[in_MOB->Hardware_buffer][4];
		in_MOB->data[3]=(unsigned char)((ecan1msgBuf[in_MOB->Hardware_buffer][4] & 0xFF00) >> 8);
		in_MOB->data[4]=(unsigned char)ecan1msgBuf[in_MOB->Hardware_buffer][5];
		in_MOB->data[5]=(unsigned char)((ecan1msgBuf[in_MOB->Hardware_buffer][5] & 0xFF00) >> 8);
		in_MOB->data[6]=(unsigned char)ecan1msgBuf[in_MOB->Hardware_buffer][6];
		in_MOB->data[7]=(unsigned char)((ecan1msgBuf[in_MOB->Hardware_buffer][6] & 0xFF00) >> 8);
		in_MOB->data_length=(unsigned char)(ecan1msgBuf[in_MOB->Hardware_buffer][2] & 0x000F);
	}
    
	clearRxFlags(in_MOB->Hardware_buffer);	
    return OK;
 } 
 
 /******************************************************************************
*                                                                             
*    Function:			clearRxFlags
*    Description:       clears the rxfull flag after the message is read
*                                                                             
*    Arguments:			buffer number to clear 
*	 Author:            Jatinder Gharoo                                                      
*	                                                                 
*                                                                              
******************************************************************************/
void clearRxFlags(unsigned char buffer_number)
{
	if((C1RXFUL1bits.RXFUL0) && (buffer_number==0))
		C1RXFUL1bits.RXFUL0=0;	
	else if((C1RXFUL1bits.RXFUL1) && (buffer_number==1))
		C1RXFUL1bits.RXFUL1=0;		
	else if((C1RXFUL1bits.RXFUL2) && (buffer_number==2))
		C1RXFUL1bits.RXFUL2=0;				
	else if((C1RXFUL1bits.RXFUL3) && (buffer_number==3))
		C1RXFUL1bits.RXFUL3=0;				
	else if((C1RXFUL1bits.RXFUL4) && (buffer_number==4))
		C1RXFUL1bits.RXFUL4=0;
    else if((C1RXFUL1bits.RXFUL5) && (buffer_number==5))
		C1RXFUL1bits.RXFUL5=0;
    else if((C1RXFUL1bits.RXFUL6) && (buffer_number==6))
		C1RXFUL1bits.RXFUL6=0;
    else if((C1RXFUL1bits.RXFUL7) && (buffer_number==7))
		C1RXFUL1bits.RXFUL7=0;
    else if((C1RXFUL1bits.RXFUL8) && (buffer_number==8))
		C1RXFUL1bits.RXFUL8=0;
    else if((C1RXFUL1bits.RXFUL9) && (buffer_number==9))
		C1RXFUL1bits.RXFUL9=0;
    else if((C1RXFUL1bits.RXFUL10) && (buffer_number==10))
		C1RXFUL1bits.RXFUL10=0;
    else if((C1RXFUL1bits.RXFUL11) && (buffer_number==11))
		C1RXFUL1bits.RXFUL11=0;
    else if((C1RXFUL1bits.RXFUL12) && (buffer_number==12))
		C1RXFUL1bits.RXFUL12=0;
    else if((C1RXFUL1bits.RXFUL13) && (buffer_number==13))
		C1RXFUL1bits.RXFUL13=0;
    else if((C1RXFUL1bits.RXFUL14) && (buffer_number==14))
		C1RXFUL1bits.RXFUL14=0;
    else if((C1RXFUL1bits.RXFUL15) && (buffer_number==15))
		C1RXFUL1bits.RXFUL15=0;
	else;
}
 
 uint_fast8_t CAN_check_new_Data (CAN_MOB *in_MOB) {
    
     if((C1RXFUL1bits.RXFUL0) && (in_MOB->Hardware_buffer==0))
		return OK;	
	else if((C1RXFUL1bits.RXFUL1) && (in_MOB->Hardware_buffer==1))
		return OK;		
	else if((C1RXFUL1bits.RXFUL2) && (in_MOB->Hardware_buffer==2))
		return OK;				
	else if((C1RXFUL1bits.RXFUL3) && (in_MOB->Hardware_buffer==3))
		return OK;				
	else if((C1RXFUL1bits.RXFUL4) && (in_MOB->Hardware_buffer==4))
		return OK;
    else if((C1RXFUL1bits.RXFUL5) && (in_MOB->Hardware_buffer==5))
		return OK;
    else if((C1RXFUL1bits.RXFUL6) && (in_MOB->Hardware_buffer==6))
		return OK;
    else if((C1RXFUL1bits.RXFUL7) && (in_MOB->Hardware_buffer==7))
		return OK;
    else if((C1RXFUL1bits.RXFUL8) && (in_MOB->Hardware_buffer==8))
		return OK;
    else if((C1RXFUL1bits.RXFUL9) && (in_MOB->Hardware_buffer==9))
		return OK;
    else if((C1RXFUL1bits.RXFUL10) && (in_MOB->Hardware_buffer==10))
		return OK;
    else if((C1RXFUL1bits.RXFUL11) && (in_MOB->Hardware_buffer==11))
		return OK;
    else if((C1RXFUL1bits.RXFUL12) && (in_MOB->Hardware_buffer==12))
		return OK;
    else if((C1RXFUL1bits.RXFUL13) && (in_MOB->Hardware_buffer==13))
		return OK;
    else if((C1RXFUL1bits.RXFUL14) && (in_MOB->Hardware_buffer==14))
		return OK;
    else if((C1RXFUL1bits.RXFUL15) && (in_MOB->Hardware_buffer==15))
		return OK;
	else;
        return Error;
 }