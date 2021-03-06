/* 
 * File:   PIC33EP_CAN.h
 * Author: maxi
 *
 * Created on 2. Mai 2019, 08:25
 */

#ifndef PIC33EP_CAN_H
#define	PIC33EP_CAN_H

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
//CAN Modes (S 297)
    /*
    enum {  Normal_Operation_mode,
            Disable_mode,
            Loopback_mode,
            Listen_Only_mode,
            Configuration_mode,
            Listen_All_Messages_mode = 7 
    };
    */
    
/* filter and mask defines */
/* Macro used to write filter/mask ID to Register CiRXMxSID and 
CiRXFxSID. For example to setup the filter to accept a value of 
0x123, the macro when called as CAN_FILTERMASK2REG_SID(0x123) will 
write the register space to accept message with ID ox123 
USE FOR STANDARD MESSAGES ONLY */
#define CAN_FILTERMASK2REG_SID(x) ((x & 0x07FF)<< 5)
    
/* the Macro will set the "MIDE" bit in CiRXMxSID */
#define CAN_SETMIDE(sid) (sid | 0x0008)
    
/* the macro will set the EXIDE bit in the CiRXFxSID to 
accept extended messages only */
#define CAN_FILTERXTD(sid) (sid | 0x0008)
    
/* the macro will clear the EXIDE bit in the CiRXFxSID to 
accept standard messages only */
#define CAN_FILTERSTD(sid) (sid & 0xFFF7)
    
/* Macro used to write filter/mask ID to Register CiRXMxSID, CiRXMxEID and 
CiRXFxSID, CiRXFxEID. For example to setup the filter to accept a value of 
0x123, the macro when called as CAN_FILTERMASK2REG_SID(0x123) will 
write the register space to accept message with ID 0x123 
USE FOR EXTENDED MESSAGES ONLY */
#define CAN_FILTERMASK2REG_EID0(x) (x & 0xFFFF)
#define CAN_FILTERMASK2REG_EID1(x) (((x & 0x1FFC)<< 3)|(x & 0x3))
    
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

#ifdef	__cplusplus
}
#endif

#endif	/* PIC33EP_CAN_H */

