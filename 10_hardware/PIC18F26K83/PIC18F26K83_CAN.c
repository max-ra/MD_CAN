#if defined(__18F26K83)
// ##### Including #####
#include "PIC18F26K83_CAN.h"

// ##### Settings #####


// ##### definition #####
	

// ##### Status definition #####
	

// ##### variable #####

/* Define ECAN Message Buffers */
uint_fast16_t Buffer_State; 


// ##### function #####
void clearRxFlags(unsigned char buffer_number);
static uint32_t convertReg2ExtendedCANid(uint8_t tempRXBn_EIDH, uint8_t tempRXBn_EIDL, uint8_t tempRXBn_SIDH, uint8_t tempRXBn_SIDL);
static uint32_t convertReg2StandardCANid(uint8_t tempRXBn_SIDH, uint8_t tempRXBn_SIDL);
static void convertCANid2Reg(CAN_MOB *in_MOB, uint8_t *passedInEIDH, uint8_t *passedInEIDL, uint8_t *passedInSIDH, uint8_t *passedInSIDL);

void CAN_init(void) {
// Clear Buffer usage
    Buffer_State = 0;
    
//Init Connection to board
   
    //Input Setup     
     ANSELBbits.ANSELB4 = 0;              //Disabel Analouge function at B3
     TRISBbits.TRISB4 = 1;              //Set as input pin
     CANRXPPS = 0x0C;                   //RB4->ECAN:CANRX;    
    
    //Output Setup
     ANSELBbits.ANSELB3 = 0;            //Disable analouge functions
     TRISBbits.TRISB3 = 0;              //TX as Output
     RB3PPS = 0x33;                     //RB3->ECAN:CANTX0;  
     
//Switch CAN Interface to Setup mode
    CANCON = 0x80;
    while (0x80 != (CANSTAT & 0xE0)); // wait until ECAN is in config mode
    

    // Mode 1 Enhanced Legacy mode (Mode 1)
    ECANCON = 0x40;
    
    // Clock source 
     CIOCON = 0x00;
     
    // Initialize Receive Masks
    RXM0EIDH = 0xFF;
    RXM0EIDL = 0xFF;
    RXM0SIDH = 0xFF;
    RXM0SIDL = 0xE3;
    RXM1EIDH = 0xFF;
    RXM1EIDL = 0xFF;
    RXM1SIDH = 0xFF;
    RXM1SIDL = 0xE3;
    
    // Assign Filters to Masks 
    MSEL0 = 0x00;
    MSEL1 = 0x00;
    MSEL2 = 0x00;
    MSEL3 = 0x00;

    
    //setup Timing
    /**	Baud rate: 1Mbps
	System frequency: 16000000
    ECAN clock frequency: 16000000
	Time quanta: 8
	Sample point: 1-1-4-2
	Sample point: 75% */
    BRGCON1 = 0x80;
    BRGCON2 = 0x98;
    BRGCON3 = 0x01;
    
    CANCON = 0x00;
    while (0x00 != (CANSTAT & 0xE0)); // wait until ECAN is in Normal mode
  
//Clear Buffer ??
  //C1RXFUL1=C1RXFUL2=C1RXOVF1=C1RXOVF2=0x0000;
  
    
//Set hardware buffer 0 to occupied (Transmit Buffer)
    Buffer_State |= (1<<0);
    
//Interupt setup
    /* configure the device to disable interrupts  */
	/* clear the buffer full flags */
    PIR5bits.TXBnIF = 0;
}

 uint_fast8_t CAN_RxMOB_init(CAN_MOB *in_MOB) {
//Check if Hardware Buffer is allready in use
     if (Buffer_State & (1<<(in_MOB->Hardware_buffer))) {
         return Error;
     }
     
     
//Switch CAN Interface to Setup mode
    CANCON = 0x80;
    while (0x80 != (CANSTAT & 0xE0)); // wait until ECAN is in config mode
          
    
if (in_MOB->frame_type==standard) {
    switch(in_MOB->Hardware_buffer) {
        case 1:
           //Set filter id
            RXF0SIDH = High(in_MOB->Identifyer.standard);
            RXF0SIDL = Low(in_MOB->Identifyer.standard & 0xE0);
    
           /* set filter to accept standard id only */
            RXF0EIDH = 0x00;
            RXF0EIDL = 0x00;
    
           /* acceptance filter 0 to use buffer RXB0 for incoming messages */
            RXFBCON0bits.F0BP = 0;
    
           /* enable filter 0 */
            RXFCON0bits.RXF0EN = 1;
        break;    
        
        case 2:
           //Set filter id
            RXF1SIDH = High(in_MOB->Identifyer.standard);
            RXF1SIDL = Low(in_MOB->Identifyer.standard & 0xE0);
    
           /* set filter to accept standard id only */
            RXF1EIDH = 0x00;
            RXF1EIDL = 0x00;
    
           /* acceptance filter 0 to use buffer RXB0 for incoming messages */
            RXFBCON0bits.F1BP = 0;
    
           /* enable filter 0 */
            RXFCON0bits.RXF1EN = 1;
        break;    
        
        case 3:
           //Set filter id
            RXF2SIDH = High(in_MOB->Identifyer.standard);
            RXF0SIDL = Low(in_MOB->Identifyer.standard & 0xE0);
    
           /* set filter to accept standard id only */
            RXF2EIDH = 0x00;
            RXF2EIDL = 0x00;
    
           /* acceptance filter 0 to use buffer RXB0 for incoming messages */
            RXFBCON1bits.F2BP = 0;
    
           /* enable filter 0 */
            RXFCON0bits.RXF2EN = 1;

        break;    
        
        case 4:
           //Set filter id
            RXF3SIDH = High(in_MOB->Identifyer.standard);
            RXF3SIDL = Low(in_MOB->Identifyer.standard & 0xE0);
    
           /* set filter to accept standard id only */
            RXF3EIDH = 0x00;
            RXF3EIDL = 0x00;
    
           /* acceptance filter 0 to use buffer RXB0 for incoming messages */
            RXFBCON1bits.F3BP = 0;
    
           /* enable filter 0 */
            RXFCON0bits.RXF3EN = 1;

        break;  
        
        case 5:
           //Set filter id
            RXF4SIDH = High(in_MOB->Identifyer.standard);
            RXF4SIDL = Low(in_MOB->Identifyer.standard & 0xE0);
    
           /* set filter to accept standard id only */
            RXF4EIDH = 0x00;
            RXF4EIDL = 0x00;
    
           /* acceptance filter 0 to use buffer RXB0 for incoming messages */
            RXFBCON2bits.F4BP = 0;
    
           /* enable filter 0 */
            RXFCON0bits.RXF4EN = 1;

        break;  
        
        case 6:
           //Set filter id
            RXF5SIDH = High(in_MOB->Identifyer.standard);
            RXF5SIDL = Low(in_MOB->Identifyer.standard & 0xE0);
    
           /* set filter to accept standard id only */
            RXF5EIDH = 0x00;
            RXF5EIDL = 0x00;
    
           /* acceptance filter 0 to use buffer RXB0 for incoming messages */
            RXFBCON2bits.F5BP = 0;
    
           /* enable filter 0 */
            RXFCON0bits.RXF5EN = 1;

        break;  
        
        default:
            CANCON = 0x00;
            while (0x00 != (CANSTAT & 0xE0)); // wait until ECAN is in Normal mode
            return Error;
        break;
    }
   
}    
    
// set hw buffer als occupied
Buffer_State |= (1<<in_MOB->Hardware_buffer);
    
   
//Set CAN Controller in normal mode
CANCON = 0x00;
while (0x00 != (CANSTAT & 0xE0)); // wait until ECAN is in Normal mode   


return OK;
 }
 
 uint_fast8_t CAN_send_Data(CAN_MOB *in_MOB) {
    uint8_t tempEIDH = 0;
    uint8_t tempEIDL = 0;
    uint8_t tempSIDH = 0;
    uint8_t tempSIDL = 0;
    uint8_t returnValue = 0;

    if (TXB0CONbits.TXREQ != 1) 
    {
        convertCANid2Reg(in_MOB, &tempEIDH, &tempEIDL, &tempSIDH, &tempSIDL);

        TXB0EIDH = tempEIDH;
        TXB0EIDL = tempEIDL;
        TXB0SIDH = tempSIDH;
        TXB0SIDL = tempSIDL;
        TXB0DLC  = in_MOB->data_length;
        TXB0D0   = in_MOB->data[0];
        TXB0D1   = in_MOB->data[1];
        TXB0D2   = in_MOB->data[2];
        TXB0D3   = in_MOB->data[3];
        TXB0D4   = in_MOB->data[4];
        TXB0D5   = in_MOB->data[5];
        TXB0D6   = in_MOB->data[6];
        TXB0D7   = in_MOB->data[7];

        TXB0CONbits.TXREQ = 1; //Set the buffer to transmit		
        returnValue = 1;
        
    } 
    else if (TXB1CONbits.TXREQ != 1) 
    {

        convertCANid2Reg(in_MOB, &tempEIDH, &tempEIDL, &tempSIDH, &tempSIDL);

        TXB1EIDH = tempEIDH;
        TXB1EIDL = tempEIDL;
        TXB1SIDH = tempSIDH;
        TXB1SIDL = tempSIDL;
        TXB1DLC  = in_MOB->data_length;
        TXB1D0   = in_MOB->data[0];
        TXB1D1   = in_MOB->data[1];
        TXB1D2   = in_MOB->data[2];
        TXB1D3   = in_MOB->data[3];
        TXB1D4   = in_MOB->data[4];
        TXB1D5   = in_MOB->data[5];
        TXB1D6   = in_MOB->data[6];
        TXB1D7   = in_MOB->data[7];

        TXB1CONbits.TXREQ = 1; //Set the buffer to transmit		
        returnValue = 1;
    } 
    else if (TXB2CONbits.TXREQ != 1) 
    {

        convertCANid2Reg(in_MOB, &tempEIDH, &tempEIDL, &tempSIDH, &tempSIDL);

        TXB2EIDH = tempEIDH;
        TXB2EIDL = tempEIDL;
        TXB2SIDH = tempSIDH;
        TXB2SIDL = tempSIDL;
        TXB2DLC  = in_MOB->data_length;
        TXB2D0   = in_MOB->data[0];
        TXB2D1   = in_MOB->data[1];
        TXB2D2   = in_MOB->data[2];
        TXB2D3   = in_MOB->data[3];
        TXB2D4   = in_MOB->data[4];
        TXB2D5   = in_MOB->data[5];
        TXB2D6   = in_MOB->data[6];
        TXB2D7   = in_MOB->data[7];

        TXB2CONbits.TXREQ = 1; //Set the buffer to transmit		
        returnValue = 1;
    }
    
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
		
 uint8_t returnValue = 0;

    //check which buffer the CAN message is in
    if ((RXB0CONbits.RXFUL != 0) && (in_MOB->Hardware_buffer == 0)) //CheckRXB0
    {
        if ((RXB0SIDL & 0x08) == 0x08) //If Extended Message
        {   
            //message is extended
            in_MOB->frame_type = extendet;
            in_MOB->Identifyer.extendet = convertReg2ExtendedCANid(RXB0EIDH, RXB0EIDL, RXB0SIDH, RXB0SIDL);
        }
        else
        {
            //message is standard
            in_MOB->frame_type = standard;
            in_MOB->Identifyer.standard = convertReg2StandardCANid(RXB0SIDH, RXB0SIDL);
        }

       
        in_MOB->data_length = RXB0DLC;
        in_MOB->data[0] = RXB0D0;
        in_MOB->data[1] = RXB0D1;
        in_MOB->data[2] = RXB0D2;
        in_MOB->data[3] = RXB0D3;
        in_MOB->data[4] = RXB0D4;
        in_MOB->data[5] = RXB0D5;
        in_MOB->data[6] = RXB0D6;
        in_MOB->data[7] = RXB0D7;
        RXB0CONbits.RXFUL = 0;
        returnValue = 1;
    }
    else if ((RXB0CONbits.RXFUL != 0) && (in_MOB->Hardware_buffer == 1)) //CheckRXB1
    {
        if ((RXB1SIDL & 0x08) == 0x08) //If Extended Message
        {
            //message is extended
            in_MOB->frame_type = extendet;
            in_MOB->Identifyer.extendet = convertReg2ExtendedCANid(RXB1EIDH, RXB1EIDL, RXB1SIDH, RXB1SIDL);
        }
        else
        {
            //message is standard
            in_MOB->frame_type = standard;
            in_MOB->Identifyer.standard = convertReg2StandardCANid(RXB1SIDH, RXB1SIDL);
        }

        in_MOB->data_length = RXB1DLC;
        in_MOB->data[0] = RXB1D0;
        in_MOB->data[1] = RXB1D1;
        in_MOB->data[2] = RXB1D2;
        in_MOB->data[3] = RXB1D3;
        in_MOB->data[4] = RXB1D4;
        in_MOB->data[5] = RXB1D5;
        in_MOB->data[6] = RXB1D6;
        in_MOB->data[7] = RXB1D7;
        RXB1CONbits.RXFUL = 0;
        returnValue = 1;
    }
    else if (B0CONbits.RXFUL_TXBIF != 0) //CheckB0
    {
        if ((B0SIDL & 0x08) == 0x08) //If Extended Message
        {
            //message is extended
            in_MOB->frame_type = extendet;
            in_MOB->Identifyer.extendet = convertReg2ExtendedCANid(B0EIDH, B0EIDL, B0SIDH, B0SIDL);
        }
        else
        {
            //message is standard
            in_MOB->frame_type = standard;
            in_MOB->Identifyer.standard = convertReg2StandardCANid(B0SIDH, B0SIDL);
        }
        
        in_MOB->data_length = B0DLC;
        in_MOB->data[0] = B0D0;
        in_MOB->data[1] = B0D1;
        in_MOB->data[2] = B0D2;
        in_MOB->data[3] = B0D3;
        in_MOB->data[4] = B0D4;
        in_MOB->data[5] = B0D5;
        in_MOB->data[6] = B0D6;
        in_MOB->data[7] = B0D7;
        B0CONbits.RXFUL_TXBIF = 0;
        returnValue = 1;
    }
    else if (B1CONbits.RXFUL_TXBIF != 0) //CheckB1
    {
        if ((B1SIDL & 0x08) == 0x08) //If Extended Message
        {
            //message is extended
            in_MOB->frame_type = extendet;
            in_MOB->Identifyer.extendet = convertReg2ExtendedCANid(B1EIDH, B1EIDL, B1SIDH, B1SIDL);
        }
        else
        {
            //message is standard
            in_MOB->frame_type = standard;
            in_MOB->Identifyer.standard = convertReg2StandardCANid(B1SIDH, B1SIDL);
        }

        in_MOB->data_length = B1DLC;
        in_MOB->data[0] = B1D0;
        in_MOB->data[1] = B1D1;
        in_MOB->data[2] = B1D2;
        in_MOB->data[3] = B1D3;
        in_MOB->data[4] = B1D4;
        in_MOB->data[5] = B1D5;
        in_MOB->data[6] = B1D6;
        in_MOB->data[7] = B1D7;

        B1CONbits.RXFUL_TXBIF = 0;
        returnValue = 1;
    }
    else if (B2CONbits.RXFUL_TXBIF != 0) //CheckB2
    {
        if ((B2SIDL & 0x08) == 0x08) //If Extended Message
        {
            //message is extended
            in_MOB->frame_type = extendet;
            in_MOB->Identifyer.extendet = convertReg2ExtendedCANid(B2EIDH, B2EIDL, B2SIDH, B2SIDL);
        }
        else
        {
            //message is standard
            in_MOB->frame_type = standard;
            in_MOB->Identifyer.standard = convertReg2StandardCANid(B2SIDH, B2SIDL);
        }

        in_MOB->data_length = B2DLC;
        in_MOB->data[0] = B2D0;
        in_MOB->data[1] = B2D1;
        in_MOB->data[2] = B2D2;
        in_MOB->data[3] = B2D3;
        in_MOB->data[4] = B2D4;
        in_MOB->data[5] = B2D5;
        in_MOB->data[6] = B2D6;
        in_MOB->data[7] = B2D7;

        B2CONbits.RXFUL_TXBIF = 0;
        returnValue = 1;
    }
    else if (B3CONbits.RXFUL_TXBIF != 0) //CheckB3
    {
        if ((B3SIDL & 0x08) == 0x08) //If Extended Message
        {
            //message is extended
            in_MOB->frame_type = extendet;
            in_MOB->Identifyer.extendet = convertReg2ExtendedCANid(B3EIDH, B3EIDL, B3SIDH, B3SIDL);
        }
        else
        {
            //message is standard
            in_MOB->frame_type = standard;
            in_MOB->Identifyer.standard = convertReg2StandardCANid(B3SIDH, B3SIDL);
        }

        in_MOB->data_length = B3DLC;
        in_MOB->data[0] = B3D0;
        in_MOB->data[1] = B3D1;
        in_MOB->data[2] = B3D2;
        in_MOB->data[3] = B3D3;
        in_MOB->data[4] = B3D4;
        in_MOB->data[5] = B3D5;
        in_MOB->data[6] = B3D6;
        in_MOB->data[7] = B3D7;

        B3CONbits.RXFUL_TXBIF = 0;
        returnValue = 1;
    }
    else if (B4CONbits.RXFUL_TXBIF != 0) //CheckB4
    {
        if ((B4SIDL & 0x08) == 0x08) //If Extended Message
        {
            //message is extended
            in_MOB->frame_type = extendet;
            in_MOB->Identifyer.extendet = convertReg2ExtendedCANid(B4EIDH, B4EIDL, B4SIDH, B4SIDL);
        }
        else
        {
            //message is standard
            in_MOB->frame_type = standard;
            in_MOB->Identifyer.standard = convertReg2StandardCANid(B4SIDH, B4SIDL);
        }

        in_MOB->data_length = B4DLC;
        in_MOB->data[0] = B4D0;
        in_MOB->data[1] = B4D1;
        in_MOB->data[2] = B4D2;
        in_MOB->data[3] = B4D3;
        in_MOB->data[4] = B4D4;
        in_MOB->data[5] = B4D5;
        in_MOB->data[6] = B4D6;
        in_MOB->data[7] = B4D7;

        B4CONbits.RXFUL_TXBIF = 0;
        returnValue = 1;
    }
    
	clearRxFlags(in_MOB->Hardware_buffer);	
    return OK;
 } 
 
 /******************************************************************************
*                                                                             
*    Function:			clearRxFlags
*    Description:       clears the rxfull flag after the message is read                                      
*	                                                                 
*                                                                              
******************************************************************************/
void clearRxFlags(unsigned char buffer_number)
{
	if((RXB0CONbits.RXFUL) && (buffer_number==0))
		RXB0CONbits.RXFUL=0;	
	else if((RXB1CONbits.RXFUL) && (buffer_number==1))
		RXB1CONbits.RXFUL=0;		
	else;
}
 
 uint_fast8_t CAN_check_new_Data (CAN_MOB *in_MOB) {
    
     if((RXB0CONbits.RXFUL) && (in_MOB->Hardware_buffer==1))
		return OK;	
	else if((RXB1CONbits.RXFUL) && (in_MOB->Hardware_buffer==2))
		return OK;		
	else if((B0CONbits.RXFUL) && (in_MOB->Hardware_buffer==3))
		return OK;				
	else if((B1CONbits.RXFUL) && (in_MOB->Hardware_buffer==4))
		return OK;				
	else if((B2CONbits.RXFUL) && (in_MOB->Hardware_buffer==5))
		return OK;
    else if((B3CONbits.RXFUL) && (in_MOB->Hardware_buffer==6))
		return OK;
    else if((B4CONbits.RXFUL) && (in_MOB->Hardware_buffer==7))
		return OK;
    else if((B5CONbits.RXFUL) && (in_MOB->Hardware_buffer==8))
		return OK;
    else
        return Error;
 }
 
uint_fast8_t CAN_check_mob_status (CAN_MOB *in_MOB) {
//switch to CAN page
if(in_MOB->Hardware_buffer > 15) {
	return Error;
}


// check if Buffer is TX and buffer 0 (workarount)
if(in_MOB->Hardware_buffer == 0) {
    if (TXB0CONbits.TXBIF == 1) {
        in_MOB->Status = TX_Finish;
    } else if ((TXB0CONbits.TXREQ)) {
        in_MOB->Status = Pending;
    }
    }
return OK;
}

uint_fast8_t CAN_finish_mob (CAN_MOB *in_MOB) {
    return OK;
}


static uint32_t convertReg2ExtendedCANid(uint8_t tempRXBn_EIDH, uint8_t tempRXBn_EIDL, uint8_t tempRXBn_SIDH, uint8_t tempRXBn_SIDL) {
    uint32_t returnValue = 0;
    uint32_t ConvertedID = 0;
    uint8_t CAN_standardLo_ID_lo2bits;
    uint8_t CAN_standardLo_ID_hi3bits;

    CAN_standardLo_ID_lo2bits = (uint8_t)(tempRXBn_SIDL & 0x03);
    CAN_standardLo_ID_hi3bits = (uint8_t)(tempRXBn_SIDL >> 5);
    ConvertedID = (uint32_t)(tempRXBn_SIDH << 3);
    ConvertedID = ConvertedID + CAN_standardLo_ID_hi3bits;
    ConvertedID = (ConvertedID << 2);
    ConvertedID = ConvertedID + CAN_standardLo_ID_lo2bits;
    ConvertedID = (ConvertedID << 8);
    ConvertedID = ConvertedID + tempRXBn_EIDH;
    ConvertedID = (ConvertedID << 8);
    ConvertedID = ConvertedID + tempRXBn_EIDL;
    returnValue = ConvertedID;
    return (returnValue);
}

static uint32_t convertReg2StandardCANid(uint8_t tempRXBn_SIDH, uint8_t tempRXBn_SIDL) {
    uint32_t returnValue = 0;
    uint32_t ConvertedID;
    //if standard message (11 bits)
    //EIDH = 0 + EIDL = 0 + SIDH + upper three bits SIDL (3rd bit needs to be clear)
    //1111 1111 111
    ConvertedID = (uint32_t)(tempRXBn_SIDH << 3);
    ConvertedID = ConvertedID + (uint32_t)(tempRXBn_SIDL >> 5);
    returnValue = ConvertedID;
    return (returnValue);
}

static void convertCANid2Reg(CAN_MOB *in_MOB, uint8_t *passedInEIDH, uint8_t *passedInEIDL, uint8_t *passedInSIDH, uint8_t *passedInSIDL) {
    uint8_t wipSIDL = 0;

    if (in_MOB->frame_type == extendet) {
/*
        //EIDL
     *passedInEIDL = 0xFF & tempPassedInID; //CAN_extendedLo_ID_TX1 = &HFF And CAN_UserEnter_ID_TX1
        tempPassedInID = tempPassedInID >> 8; //CAN_UserEnter_ID_TX1 = CAN_UserEnter_ID_TX1 >> 8

        //EIDH
        *passedInEIDH = 0xFF & tempPassedInID; //CAN_extendedHi_ID_TX1 = &HFF And CAN_UserEnter_ID_TX1
        tempPassedInID = tempPassedInID >> 8; //CAN_UserEnter_ID_TX1 = CAN_UserEnter_ID_TX1 >> 8

        //SIDL
        //push back 5 and or it
        wipSIDL = 0x03 & tempPassedInID;
        tempPassedInID = tempPassedInID << 3; //CAN_UserEnter_ID_TX1 = CAN_UserEnter_ID_TX1 << 3
        wipSIDL = (0xE0 & tempPassedInID) + wipSIDL;
        wipSIDL = (uint8_t)(wipSIDL + 0x08); // TEMP_CAN_standardLo_ID_TX1 = TEMP_CAN_standardLo_ID_TX1 + &H8
        *passedInSIDL = (uint8_t)(0xEB & wipSIDL); //CAN_standardLo_ID_TX1 = &HEB And TEMP_CAN_standardLo_ID_TX1

        //SIDH
        tempPassedInID = tempPassedInID >> 8;
        *passedInSIDH = 0xFF & tempPassedInID; */
    } else //(canIdType == dSTANDARD_CAN_MSG_ID_2_0B)
    {
        *passedInEIDH = 0;
        *passedInEIDL = 0;
        *passedInSIDL = Low(in_MOB->Identifyer.standard & 0xE0);
        *passedInSIDH = High(in_MOB->Identifyer.standard);
    }
}

#endif