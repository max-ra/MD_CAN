/* 
 * File:   MD_CAN.h
 * Author: maxi
 *
 * Created on 2. Mai 2019, 08:22
 */

#ifndef MD_CAN_H
#define	MD_CAN_H

#ifdef	__cplusplus
extern "C" {
#endif

// ##### Including #####
#include "common.h"
#include "../../02_kernel/scheduler/message_query.h"   
#include "../../02_kernel/scheduler/task.h"  
    
/** Switching between different Hardware version */

#if defined(__AVR_AT90CAN128__)
#include "10_hardware/AT90CAN/AT90CAN_CAN.h"
#endif
    
#if defined(__dsPIC33EP512GM706__) || (__dsPIC33EP512GM604__)
#include "10_hardware/PIC33EP/PIC33EP_CAN.h"
#endif
    
// ##### Settings #####
#define MD_CAN_Setting_MSQ_max 5

// ##### definition #####
    
    
    //Version mit Pointer 
    /*
    typedef struct{
        CAN_MOB *volatile CAN_mob;
    }MD_CAN_MSQ_Type;*/
    
    typedef CAN_MOB MD_CAN_MSQ_Type;

// ##### Status definition #####
	

// ##### variable #####

   //MSQ System
    extern EVENT_QUEUE MD_CAN_MSQ_Queue;

   //Task System
    extern Task_Status_Byte MD_CAN_Inbound_Task;
    extern Task_Status_Byte MD_CAN_Outbound_Task;
    
// ##### function #####

    void MD_CAN_init(void);
    void MD_CAN_Cycle(void);
    uint_fast8_t MD_CAN_RxMob_add(CAN_MOB *in_MOB);
    
#ifdef	__cplusplus
}
#endif

#endif	/* MD_CAN_H */

