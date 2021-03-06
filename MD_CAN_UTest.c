/*
 * File:   MD_CAN_UTest.c
 * Author: maxi
 *
 * Created on 26. Mai 2019, 16:40
 */

// ##### Including #####
#include "MD_CAN_UTest.h"
#include "MD_CAN.h"

// ##### Settings #####


// ##### definition #####
	

// ##### Status definition #####
	

// ##### variable #####
CAN_MOB UTest_TX_MOB;
CAN_MOB UTest_RX_MOB;

//MD_CAN_MSQ_Type CAN_MSQ;

// ##### function #####

void MD_CAN_UTest_prepare(void) {
//Init CAN
    //MD_CAN_init();
    
//setup MOB
    UTest_TX_MOB.Command = CMD_TX_DATA;
    UTest_TX_MOB.Hardware_buffer = 0;
    UTest_TX_MOB.Identifyer.standard = 0x9;
    UTest_TX_MOB.data_length = 8;
    UTest_TX_MOB.frame_type = standard;
    
    UTest_RX_MOB.Hardware_buffer = 1;
    UTest_RX_MOB.Identifyer.standard = 0x02;
    UTest_RX_MOB.frame_type= standard;
    UTest_RX_MOB.Command = CMD_RX_DATA;
    UTest_RX_MOB.work_don_callback = &MD_CAN_UTest_helper_rx_mob_callback;
}

void MD_CAN_UTest_assert_add_MOB(void) {
    MD_CAN_RxMob_add(&UTest_RX_MOB);
}

void MD_CAN_UTest_helper_rx_mob_callback (void) {
    while (1) {

    }

}

void MD_CAN_UTest_assert_transmitt_MOB(void) {
    UTest_TX_MOB.data[0] = 1;
    UTest_TX_MOB.data[1] = 10;
    UTest_TX_MOB.data[2] = 100;
    UTest_TX_MOB.data[3] = 200;
    
    MSQ_message_push(&MD_CAN_MSQ_Queue,&UTest_TX_MOB);
    
}