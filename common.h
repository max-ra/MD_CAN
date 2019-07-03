/* 
 * File:   common.h
 * Author: maxi
 *
 * Created on 4. Mai 2019, 10:52
 */

#ifndef COMMON_H
#define	COMMON_H

#ifdef	__cplusplus
extern "C" {
#endif

// ##### Including #####
#include "../../global.h"

    
// ##### Settings #####


// ##### definition #####
    // ----------
// @brief This enumeration is used to select an action for a specific message
// declared in st_cmd_t structure.
typedef enum {
  CMD_NONE,
  //CMD_TX,
  CMD_TX_DATA,
  CMD_TX_REMOTE,
  //CMD_RX, 
  CMD_RX_DATA, 
  CMD_RX_REMOTE,
  //CMD_RX_MASKED,
  //CMD_RX_DATA_MASKED,
  //CMD_RX_REMOTE_MASKED, 
  CMD_REPLY,        
  //CMD_REPLY_MASKED,
  CMD_ABORT
} CAN_cmd; 
	

typedef enum {
    standard,
    extendet
} CAN_frame_type;

typedef enum {
   Empty,           /**< No new data availabel.*/
   New_Data,
   Data_Sent,
   Reading,
   RX_Finish,           /**<  Reading is finished and new CAN Data can feed in the buffer.*/
   TX_Finish,
   Pending,			/**< Wait for HW to transmitt or recive.*/
   Fault
}CAN_Stat;

typedef union {
    uint_fast32_t extendet;
    uint_fast16_t standard;
} CAN_id;

/* message structure in RAM */
typedef struct{
	CAN_Stat Status;
	CAN_cmd Command;
	CAN_frame_type frame_type;
	uint_fast8_t Hardware_buffer;   	/**< Number of Hardware DMA buffer (Message Box)used to send and receive messages */
	CAN_id Identifyer; 
	unsigned char data[8];	
	unsigned char data_length;
    void (*work_don_callback) (void);
}CAN_MOB;
	

// ##### Status definition #####
	

// ##### variable #####


// ##### function #####



#ifdef	__cplusplus
}
#endif

#endif	/* COMMON_H */

