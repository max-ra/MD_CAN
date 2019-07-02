/*
 * AT90CAN_CAN.c
 *
 * Created: 30.06.2019 22:49:05
 *  Author: maxi
 */ 

// ##### Including #####
#include "AT90CAN_CAN.h"
#include "../../../../02_kernel/hardware/common/Interrupt.h"

// ##### Settings #####


// ##### definition #####


// ##### Status definition #####


// ##### variable #####


// ##### function #####


void CAN_init(void) {
if ((Can_bit_timing(mode))==0) {
	return Error;
}

can_clear_all_mob();
Can_enable();  

return OK;
}

void can_clear_all_mob(void)
{
uint8_t  mob_number;
uint8_t  data_index;

	for (mob_number = 0; mob_number < NB_MOB; mob_number++)
    {
        CANPAGE = (mob_number << 4);    //! Page index
        Can_clear_mob();                //! All MOb Registers=0

        for (data_index = 0; data_index < NB_DATA_MAX; data_index++)
        {
            CANMSG = 0;                 //! MOb data FIFO
        }
    }
}

uint8_t can_get_mob_free(void)
{
    uint8_t mob_number, page_saved;

    page_saved = CANPAGE;
    for (mob_number = 0; mob_number < NB_MOB; mob_number++)
    {
        Can_set_mob(mob_number);
        if ((CANCDMOB & 0xC0) == 0x00) //! Disable configuration
        {
            CANPAGE = page_saved;
            return (mob_number);
        }
    }
    CANPAGE = page_saved;
    return (NO_MOB);
}

uint8_t can_get_mob_status(void) {
	uint8_t mob_status, canstmob_copy;

	// Test if MOb ENABLE or DISABLE
	if ((CANCDMOB & 0xC0) == 0x00) {return(MOB_DISABLE);}

	canstmob_copy = CANSTMOB; // Copy for test integrity

	// If MOb is ENABLE, test if MOb is COMPLETED
	// - MOb Status = 0x20 then MOB_RX_COMPLETED
	// - MOb Status = 0x40 then MOB_TX_COMPLETED
	// - MOb Status = 0xA0 then MOB_RX_COMPLETED_DLCW
	mob_status = canstmob_copy & ((1<<DLCW)|(1<<TXOK)|(1<<RXOK));
	if ( (mob_status==MOB_RX_COMPLETED) ||   \
	(mob_status==MOB_TX_COMPLETED) ||   \
	(mob_status==MOB_RX_COMPLETED_DLCW) ) { return(mob_status); }

	// If MOb is ENABLE & NOT_COMPLETED, test if MOb is in ERROR
	// - MOb Status bit_0 = MOB_ACK_ERROR
	// - MOb Status bit_1 = MOB_FORM_ERROR
	// - MOb Status bit_2 = MOB_CRC_ERROR
	// - MOb Status bit_3 = MOB_STUFF_ERROR
	// - MOb Status bit_4 = MOB_BIT_ERROR
	mob_status = canstmob_copy & ERR_MOB_MSK;
	if (mob_status != 0) { return(mob_status); }

	// If CANSTMOB = 0 then MOB_NOT_COMPLETED
	return(MOB_NOT_COMPLETED);
}

uint_fast8_t  CAN_recive_Data(CAN_MOB *in_MOB)
{

//switch to CAN page
 if((in_MOB->Hardware_buffer == 0) || (in_MOB->Hardware_buffer > 14)) {
	 return Error;
 }
 CANPAGE = (in_MOB->Hardware_buffer << 4);

 interrupt_disable();
	uint8_t data_index;
	#if CAN_setting_swap_endianness
	 int_fast8_t swap_bytes = 1;
	#else
	 int_fast8_t swap_bytes = 0;
	#endif

	for (data_index = 0; data_index < (Can_get_dlc()); data_index++)
	{
		in_MOB->data[data_index+swap_bytes] = CANMSG;
		
		#if CAN_setting_swap_endianness
		 if (swap_bytes == 1)
		  swap_bytes = -1;
		 else
		  swap_bytes = 1;
		#endif
	}

	interrupt_enable();
	
	return OK;
}


uint8_t can_auto_baudrate (uint8_t mode)
{
	uint8_t  uint8_t_temp0;                               //! Temporary variable
	uint8_t  brp, prs, ntq, phs1, phs2;              //! Bit timing segment variables
	uint8_t  phs1_inc;                               //! Computing needed
	uint8_t  bt_not_found, wait_for_rx, evaluate;    //! Keys for "while()" loops
	uint8_t  try_conf;                               //! Key for configurate CAN
	uint8_t  ovrtim_flag=0;                          //! Timer overflow count
	U16 conf_index;                             //! Count of bit timing configuration tried
	uint8_t  bt_performed;                           //! Return flag

	//! --- Default setting
	phs1_inc = evaluate = 0;
	bt_performed = 0;
	conf_index = 0;
	bt_not_found = 1;

	//! --- Init segment variables with MIN values if mode=0
	//!     or init segment variables with CANBTx if mode=1
	if (mode==0)
	{
		brp  = BRP_MIN;
		ntq  = NTQ_MIN;
		phs1 = PHS1_MIN;
		phs2 = PHS2_MIN;
		prs  = ntq - ( phs1 + phs2 + 1 );
		try_conf = 1;       //! Try this configuration
		wait_for_rx = 1;    //! Enable "while (wait_for_rx ..." loop
	}
	else //! mode = 1
	{
		brp  = Max ((((CANBT1 &  BRP_MSK) >> 1) +1) , BRP_MIN );
		prs  = Max ((((CANBT2 &  PRS_MSK) >> 1) +1) , PRS_MIN );
		phs1 = Max ((((CANBT3 & PHS1_MSK) >> 1) +1) , PHS1_MIN);
		phs2 = Max ((((CANBT3 & PHS2_MSK) >> 4) +1) , PHS2_MIN);
		ntq  = Max ((prs + phs1 + phs2 + 1) , NTQ_MIN);
		phs1_inc = evaluate = 1;   //! To enter in "while (evaluate ..." loop
		try_conf = 0;       //! Look for the next configuration
		wait_for_rx = 0;    //! Skip "while (wait_for_rx ..." loop
	}

	//! --- Clear all MOb's (CANMSG not cleared)
	for (uint8_t_temp0 = 0; uint8_t_temp0 < NB_MOB; uint8_t_temp0++)
	{
		Can_set_mob(uint8_t_temp0);  //! Page index
		Can_clear_mob();        //! All MOb Registers = 0x00
	}

	while (bt_not_found == 1)
	{
		if (try_conf == 1)
		{
			Can_reset();
			conf_index++;
			ovrtim_flag=0;

			//! --- CANBTx registers update (sjw = phs2/2, 3 sample points)
			CANBT1 = ((brp-1) << BRP);
			CANBT2 = (((phs2 >> 1)-1) << SJW) |((prs-1) << PRS);
			CANBT3 = (((phs2-1) << PHS2) | ((phs1-1) << PHS1) | (1<<SMP));

			//! --- Set CAN-Timer - Used for time-out
			//!     There are 641 (0x281) possible evaluations. The first one provides the faster
			//!         the faster bit timing, the last one gives the slower. It is necessary to
			//!         modulate the time-out versus bit timing (0x281>>3=0x50, matching an uint8_t).
			CANTCON = (uint8_t)(conf_index >> 3);

			//! --- MOb configuration
			Can_set_mob(MOB_0);                 //! Use MOb-0
			CANSTMOB = 0;                       //! Reset MOb status (undone by "Can_reset()")
			CANCDMOB = (MOB_Rx_ENA  << CONMOB); //! MOb 0 in receive mode

			//! CAN controller configuration
			CANGCON = (1<<LISTEN) | (1<<ENASTB);//! Enable CAN controller in "listen" mode
			while ((CANGSTA & (1<<ENFG)) == 0); //! Wait for Enable OK
			CANGIT = 0xFF;                      //! Reset General errors and OVRTIM flag
		}

		//! --- WAIT_FOR_RX LOOP:
		//!     ================
		//!     Try to perform a CAN message reception in "LISTEN" mode without error and
		//!     before a time_out done by CAN-Timer.
		//!     Else gives the hand to "EVALUATE LOOP" to have a new set of bit timing.
		while (wait_for_rx == 1)
		{
			uint8_t_temp0 = CANSTMOB;
			//! --- RxOK received ?
			if ((uint8_t_temp0 & (1<<RXOK)) != 0)
			{   //! --- It is the successful output of "can_auto_baudrate" function
				wait_for_rx = 0;    //! Out of "while (wait_for_rx ..." loop
				evaluate = 0;       //! Will skip "while (evaluate ..." loop
				bt_not_found = 0;   //! Out of "while (bt_not_found ..." loop
				bt_performed = 1;   //! Return flag = TRUE
				DISABLE_MOB;        //! Disable MOb-0
				CANGCON = 0x00;     //! Disable CAN controller & reset "listen" mode
				while ((CANGSTA & (1<<ENFG)) != 0); //! Wait for Disable OK
			}
			//! --- Else stop if any errors
			else
			{
				//! --- MOb error ?
				if ((uint8_t_temp0 & ((1<<BERR)|(1<<SERR)|(1<<CERR)|(1<<FERR)|(1<<AERR))) !=0)
				{
					evaluate = 1;       //! Will enter in "while (evaluate ..." loop
					wait_for_rx = 0;    //! Out of "while (wait_for_rx ..." loop
				}

				uint8_t_temp0 = CANGIT;

				//! --- Time_out reached ?
				if ((uint8_t_temp0 & (1<<OVRTIM)) !=0 )
				{
					if (ovrtim_flag==0)
					{
						//! --- First Time_out
						CANGIT |= (1<<OVRTIM);  // Reset OVRTIM
						ovrtim_flag++;
					}
					else
					{
						//! --- Second Time_out
						CANGIT |= (1<<OVRTIM);  // Reset OVRTIM
						evaluate = 1;           //! Will enter in "while (evaluate ..." loop
						wait_for_rx = 0;        //! Out of "while (wait_for_rx ..." loop
					}
				}

				//! --- General error ?
				if ((uint8_t_temp0 & ((1<<SERG)|(1<<CERG)|(1<<FERG)|(1<<AERG))) !=0)
				{
					evaluate = 1;       //! Will enter in "while (evaluate ..." loop
					wait_for_rx = 0;    //! Out of "while (wait_for_rx ..." loop
					try_conf = 1;       //! Try this configuration
				}
			}
		} // while (wait_for_rx ...

		//! --- EVALUATE LOOP:
		//!     =============
		//!     Compute a new bit timing configuration. First, Phase 1 is increased,
		//!     then Phase2=Phase1 and if Phase1>5, Phase1 can be equal to Phase2 or
		//!     Phase2+1. After this, the number of TQ is increased up to its high
		//!     limit and after it is the Prescaler. During the computing high (80%)
		//!     and low (75%) limits of sampling point location are tested. SJW and
		//!     the number of sampling points are not calculated in this loop.
		while (evaluate == 1)
		{
			if (phs1_inc != 0) phs1++;
			phs1_inc = 1;

			// --- The following test takes into account the previous incrementation of phs1
			if ((phs1 > PHS1_MAX) && (phs2 >= PHS2_MAX))
			{
				phs1 = PHS1_MIN;
				phs2 = PHS2_MIN;
				phs1_inc = 0;
				if (ntq != NTQ_MAX) ntq++;
				else
				{
					ntq = NTQ_MIN;
					if (brp != BRP_MAX) brp++;
					else
					{
						//! --- It is the failing of "can_auto_baudrate" function
						evaluate = 0;       //! Out of "while (evaluate ..." loop
						bt_performed = 0;   //! Return flag = FALSE
						bt_not_found = 0;   //! Out of "while (bt_not_found ..." loop
						DISABLE_MOB;        //! Disable MOb-0
						CANGCON = 0x00;     //! Disable CAN controller & reset "listen" mode
						while ((CANGSTA & (1<<ENFG)) != 0); //! Wait for Disable OK
					}
				}
			}
			else    // if (phs1 > PHS1_MAX ...
			{
				//! --- If psh1 > 5 then phs1 =phs2 or =phs2+1, else phs1=phs2
				if (phs1>5)
				{
					if (phs1>(phs2+1)) phs1=(++phs2);
				}
				else
				{
					phs2=phs1;
				}
				prs = ntq - ( phs1 + phs2 + 1 );

				//! --- Test PRS limits
				if ((prs <= PRS_MAX) && (prs >= PRS_MIN))
				{
					//! --- Values  accepted if  80% >= sampling point >= 75%
					if (((phs2<<2) >= (1+prs+phs1)) && ((phs2+phs2+phs2) <= (1+prs+phs1)))
					{
						evaluate = 0;     //! Out of "while (evaluate ..." loop &
						wait_for_rx = 1;  //!    new "while (bt_not_found ..." loop
					}
				}
			}
		} // while (evaluate ...
	} // while (bt_not_found ...

	return (bt_performed);
}

uint8_t can_fixed_baudrate(uint8_t mode)
{
	Can_reset();
	Can_conf_bt();
	return 1;
}

//------------------------------------------------------------------------------
//  @fn can_cmd
//!
//! This function takes a CAN descriptor, analyses the action to do:
//! transmit, receive or abort.
//! This function returns a status (CAN_CMD_ACCEPTED or CAN_CMD_REFUSED) if
//! a MOb for Rx or Tx has been found. If no MOB has been found, the
//! application must be retry at a later date.
//! This function also updates the CAN descriptor status (MOB_PENDING or
//! MOB_NOT_REACHED) if a MOb for Rx or Tx has been found. If aborting
//! is performed, the CAN descriptor status will be set to STATUS_CLEARED.
//!
//! @param  st_cmd_t* - Can_descriptor pointer on CAN descriptor structure
//!         to select the action to do.
//!
//! @return CAN_CMD_ACCEPTED - command is accepted
//!         CAN_CMD_REFUSED  - command is refused
//!
//------------------------------------------------------------------------------
uint8_t can_cmd(CAN_MOB* in_MOB)
{
	uint8_t mob_handle, cpt;
	uint32_t uint32_t_temp;
	
//Check canpage boundary
if(in_MOB->Hardware_buffer > 14) {
	return Error;
}
	
	if (in_MOB->Command == CMD_ABORT)
	{
		if (in_MOB != Empty)
		{
			//todo wann soll abgebrochen werden??
			Can_set_mob(in_MOB->Hardware_buffer);
			Can_mob_abort();
			Can_clear_status_mob();       // To be sure !
			in_MOB->Hardware_buffer = 255;
		}
		in_MOB->Status = Empty;
	}
	else
	{
		mob_handle = can_get_mob_free();
		if (mob_handle!= NO_MOB)
		{
			//not suported by MD_CAN structure cmd->status = MOB_PENDING;
			in_MOB->Hardware_buffer = mob_handle;
			Can_set_mob(mob_handle);
			Can_clear_mob();
			
			switch (in_MOB->Command)
			{
				//------------
				case CMD_TX_DATA:
				if (in_MOB->frame_type == extendet){ 
					Can_set_ext_id(in_MOB->Identifyer.extendet);
				} else {
					Can_set_std_id(in_MOB->Identifyer.standard);
				}
				can_set_data(in_MOB->data, in_MOB->data_length);
				//for (cpt=0;cpt<cmd->dlc;cpt++) CANMSG = *(cmd->pt_data + cpt);
				Can_clear_rtr();
				Can_set_dlc(in_MOB->data_length);
				Can_config_tx();
				break;
				//------------
				case CMD_TX_REMOTE:
				if (in_MOB->frame_type == extendet){
					Can_set_ext_id(in_MOB->Identifyer.extendet);
					} else {
					Can_set_std_id(in_MOB->Identifyer.standard);
				}
				Can_set_rtr();
				Can_set_dlc(in_MOB->data_length);
				Can_config_tx();
				break;
				//------------
				case CMD_RX_DATA:
				uint32_t_temp=0; Can_set_ext_msk(uint32_t_temp);
				Can_set_dlc(in_MOB->data_length);
				Can_set_rtrmsk(); 
				Can_clear_rtr();
				Can_clear_idemsk();
				Can_config_rx();
				break;
				//------------
				case CMD_RX_REMOTE:
				uint32_t_temp=0; Can_set_ext_msk(uint32_t_temp);
				Can_set_dlc(in_MOB->data_length);
				Can_set_rtrmsk(); 
				Can_set_rtr();
				Can_clear_rplv();
				Can_clear_idemsk();
				Can_config_rx();
				break;
				//------------
				case CMD_RX_MASKED:
				if (in_MOB->frame_type == extendet){
					Can_set_ext_id(in_MOB->Identifyer.extendet);
					} else {
					Can_set_std_id(in_MOB->Identifyer.standard);
				}
				uint32_t_temp=~0; Can_set_ext_msk(uint32_t_temp);
				Can_set_dlc(in_MOB->data_length);
				Can_clear_rtrmsk();
				Can_set_idemsk();
				Can_config_rx();
				break;
				//------------
				case CMD_RX_DATA_MASKED:
				if (in_MOB->frame_type == extendet){
					Can_set_ext_id(in_MOB->Identifyer.extendet);
					} else {
					Can_set_std_id(in_MOB->Identifyer.standard);
				}
				uint32_t_temp=~0; Can_set_ext_msk(uint32_t_temp);
				Can_set_dlc(in_MOB->data_length);
				Can_set_rtrmsk(); 
				Can_clear_rtr();
				Can_set_idemsk();
				Can_config_rx();
				break;
				//------------
				case CMD_RX_REMOTE_MASKED:
				if (in_MOB->frame_type == extendet){
					Can_set_ext_id(in_MOB->Identifyer.extendet);
					} else {
					Can_set_std_id(in_MOB->Identifyer.standard);
				}
				uint32_t_temp=~0; Can_set_ext_msk(uint32_t_temp);
				Can_set_dlc(in_MOB->data_length);
				Can_set_rtrmsk();
				Can_set_rtr();
				Can_clear_rplv();
				Can_set_idemsk();
				Can_config_rx();
				break;
				//------------
				case CMD_REPLY:
				for (cpt=0;cpt<cmd->dlc;cpt++) CANMSG = *(cmd->pt_data + cpt);
				uint32_t_temp=0; Can_set_ext_msk(uint32_t_temp);
				Can_set_dlc(cmd->dlc);
				cmd->ctrl.rtr=1; Can_set_rtrmsk(); Can_set_rtr();
				Can_set_rplv();
				Can_clear_idemsk();
				Can_config_rx();
				break;
				//------------
				case CMD_REPLY_MASKED:
				if (cmd->ctrl.ide){ Can_set_ext_id(cmd->id.ext);}
				else              { Can_set_std_id(cmd->id.std);}
				for (cpt=0;cpt<cmd->dlc;cpt++) CANMSG = *(cmd->pt_data + cpt);
				uint32_t_temp=~0; Can_set_ext_msk(uint32_t_temp);
				Can_set_dlc(cmd->dlc);
				cmd->ctrl.rtr=1; Can_set_rtrmsk(); Can_set_rtr();
				Can_set_rplv();
				Can_set_idemsk();
				Can_config_rx();
				break;
				//------------
				default:
				// case CMD_NONE or not implemented command
				cmd->status = STATUS_CLEARED;
				break;
				//------------
			} // switch (cmd ...
		} // if (mob_handle ...
		else
		{
			return Error;
		}
	} // else of no CMD_ABORT
	return OK;
}