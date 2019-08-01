/* 
 * File:   PIC33EP_CAN.h
 * Author: maxi
 *
 * Created on 2. Mai 2019, 08:25
 */

#ifndef PIC18F26K83_CAN_H
#define	PIC18F26K83_CAN_H

#ifdef	__cplusplus
extern "C" {
#endif
// ##### Including #####
#include <xc.h>
#include "../../common.h"

// ##### Settings #####
#define CAN_Setting_MessageBox_Max 16
#define ECAN1_MSG_BUF_LENGTH 	4
    
// ##### definition #####
   /*
    enum {  Normal_Operation_mode,
            Disable_mode,
            Loopback_mode,
            Listen_Only_mode,
            Configuration_mode,
            Listen_All_Messages_mode = 7 
    };
    */
    
// ##### Status definition #####
	

// ##### variable #####
/* DMA CAN Message Buffer Configuration */
typedef unsigned int ECAN1MSGBUF [ECAN1_MSG_BUF_LENGTH][8];
__eds__ extern ECAN1MSGBUF  ecan1msgBuf __attribute__((eds));

// ##### function #####
    void CAN_init (void);
    uint_fast8_t CAN_RxMOB_init(CAN_MOB *in_MOB);         /**< Add new MOB to Hardware configuration. Return OK if MOB is sendet otherwise returne ERROR.*/
    uint_fast8_t CAN_send_Data(CAN_MOB *in_MOB);          /**< Return OK if MOB is sendet otherwise returne ERROR.*/
    uint_fast8_t CAN_recive_Data(CAN_MOB *in_MOB);        /**< Coppy Data from DMA to MOB buffer. Return OK if MOB is sendet otherwise returne ERROR.*/  
    uint_fast8_t CAN_check_new_Data (CAN_MOB *in_MOB);    /**< Return OK if a new CAN MSQ is recived for the specific buffer otherwise ERROR*/    
    uint_fast8_t CAN_check_mob_status (CAN_MOB *in_MOB);  /**< Check mob status and updates in MOB structure.*/
    uint_fast8_t CAN_finish_mob (CAN_MOB *in_MOB);        /**< Release resources and finish MOB.*/

#ifdef	__cplusplus
}
#endif

#endif	/* PIC18_CAN_H */

