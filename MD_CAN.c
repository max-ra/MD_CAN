// ##### Including #####
#include "MD_CAN.h"

// ##### Settings #####


// ##### definition #####
	

// ##### Status definition #####
	

// ##### variable #####

/** CAN Pointer Interface to bridge Application code to Hardwer code. 
 Mapping Message Boxes to MOB structure.*/
    static CAN_MOB *volatile Hardware_CAN_MOB[CAN_Setting_MessageBox_Max];
    static CAN_MOB CAN_MOB_dummy; 
    static uint_fast8_t Inbound_MOB_HW_Buffer_selected;
    
//MSQ Event System
    EVENT_QUEUE MD_CAN_MSQ_Queue;
    MD_CAN_MSQ_Type MD_CAN_MSQ_Buffer[MD_CAN_Setting_MSQ_max];
    static MD_CAN_MSQ_Type MD_CAN_MSQ_local;

//Task System
    Task_Status_Byte MD_CAN_Inbound_Task;
    Task_Status_Byte MD_CAN_Outbound_Task;
    static uint_fast8_t Worker_Inbound_Step;
    static uint_fast8_t Worker_Outbound_Step;
    
// ##### function #####
    void MD_CAN_worker_inbound(void);
    void MD_CAN_worker_outbound(void);
    uint_fast8_t get_free_buffer(uint_fast8_t *Buffer);
    void MD_CAN_next_mob_pointer(void);
    void MD_CAN_Callback_dummy(void);
    
    void MD_CAN_init(void) {
    //Message Boxes Mapper
        CAN_MOB_dummy.Command = CMD_NONE;
        CAN_MOB_dummy.Hardware_buffer = 0xFF;
        CAN_MOB_dummy.Identifyer.standard =  0;
        CAN_MOB_dummy.Status = Empty;
        CAN_MOB_dummy.data_length = 0;
        CAN_MOB_dummy.frame_type = standard;
        CAN_MOB_dummy.work_don_callback = &MD_CAN_Callback_dummy;
        
        uint_fast8_t i;
        for(i = 0; i < CAN_Setting_MessageBox_Max; i++){
            Hardware_CAN_MOB[i] = &CAN_MOB_dummy;
        }
        
    //MSQ System
        MSQ_EventQueue_init(&MD_CAN_MSQ_Queue, &MD_CAN_MSQ_Buffer, sizeof(MD_CAN_MSQ_Type), MD_CAN_Setting_MSQ_max);
        
    //Hardware init
        CAN_init();
        
    //Starte Task System
        MD_CAN_Outbound_Task.TaskStatus = IDLE;
        MD_CAN_Inbound_Task.TaskStatus = IDLE;
    }
    
    void MD_CAN_Cycle(void) {
        switch(MD_CAN_Inbound_Task.TaskStatus) {
            case STOP:
            case PAUSE:
            break;
                
            
            case IDLE:
                
                //Check if MOB is Reciver and Buffe is linked to real MOB and Data rad is finished or Empty
                if ((Hardware_CAN_MOB[Inbound_MOB_HW_Buffer_selected]->Command == CMD_RX_DATA ) && !(Hardware_CAN_MOB[Inbound_MOB_HW_Buffer_selected]->Status == Reading) ) {
                   //Check If new Data is arrived at CAN input Terminal
                    if ((CAN_check_new_Data(Hardware_CAN_MOB[Inbound_MOB_HW_Buffer_selected])) == OK ) {
                       //COPPY MOB DATA
                        MD_CAN_Inbound_Task.TaskStatus = RUN;
                        break;
                    }
                }
                
                MD_CAN_next_mob_pointer();
            break;
            
            case RUN:
                MD_CAN_worker_inbound();
            break;
            
            default:
                MD_CAN_Inbound_Task.TaskStatus = IDLE;
            break;
                
        }
        
        switch(MD_CAN_Outbound_Task.TaskStatus) {
            case STOP:
            case PAUSE:
            break;
                
            case IDLE:
                if ((MSQ_message_get(&MD_CAN_MSQ_Queue,&MD_CAN_MSQ_local)) == Error) {
                    break;
                } else {
                    MD_CAN_Outbound_Task.TaskStatus = RUN;
                }

            case RUN:
                MD_CAN_worker_outbound();
            break;
            
            default:
                MD_CAN_Outbound_Task.TaskStatus = IDLE;
            break;
                
        }
    }
    
    /**\TODO: Outbound Retry limit and Retry timer, Start Transmission Calback, Stop Transmision callback*/
    void MD_CAN_worker_outbound(void) {
        if ((CAN_send_Data(&MD_CAN_MSQ_local)) == OK) {
            MD_CAN_Outbound_Task.TaskStatus = IDLE;
        }
    }
    
    /**\TODO: Outbound Retry limit and Retry timer, Start Transmission Calback, Stop Transmision callback*/
    void MD_CAN_worker_inbound(void) {
        if ((CAN_recive_Data(Hardware_CAN_MOB[Inbound_MOB_HW_Buffer_selected])) == OK) {
            Hardware_CAN_MOB[Inbound_MOB_HW_Buffer_selected]->Status = New_Data;
            MD_CAN_next_mob_pointer();
            MD_CAN_Inbound_Task.TaskStatus = IDLE;
        }
    }
    
    void MD_CAN_next_mob_pointer(void) {
        //Move Inbound MOB Pointer 
        if (Inbound_MOB_HW_Buffer_selected < CAN_Setting_MessageBox_Max - 1) {
            Inbound_MOB_HW_Buffer_selected++;
        } else {
            Inbound_MOB_HW_Buffer_selected = 0;
        }
    }
    
    uint_fast8_t MD_CAN_RxMob_add(CAN_MOB *in_MOB) {
       //Check for free buffer
        uint_fast8_t Buffer;
        if (!(get_free_buffer(&Buffer) == OK)) {
            return overflow;
        }
        
       //Add MOB to Chip Hardware
        if (!(CAN_RxMOB_init(in_MOB) == OK)) {
            return Error;
        }
        
       //Add MOB to Buffer finaly
        Hardware_CAN_MOB[Buffer] = in_MOB;
        return OK;
    }
    
    
    uint_fast8_t get_free_buffer(uint_fast8_t *Buffer) {
        //Check if space is left at HW Buffer (Message Boxes))
        uint_fast8_t i;
        for (i = 0; i < CAN_Setting_MessageBox_Max; i++) {
            if (Hardware_CAN_MOB[i]->Command == CMD_NONE) {
                *Buffer = i;
                return OK;
            }
        }
        
        return Error;
    }
    
    void MD_CAN_Callback_dummy(void) {
        asm("NOP");
    }