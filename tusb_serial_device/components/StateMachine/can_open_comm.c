/*
 * can_open_comm.h
 *
 *  Created on: 11.07.2022
 *      Author: BC
 * This script is canOpen master function related to the Kinco motor driver. 
 * 
 */

#include "can_open_comm.h"

// 


/**
 * @brief Process received data
 *
 *
 * @param[twai_message_t ] *can_rx: address of the received data. 
 */
void processRxMsg(twai_message_t *can_rx_msg, motor_status_t *motor_status )
{
    uint8_t node_id  =  can_rx_msg->identifier && 0x7F; 
    
    // 功能码
    uint8_t function_code = can_rx_msg->identifier >> 7 ; 

    bytes_int_conv byte_converter; 
    byte_converter.value = 0 ; 
    sdo_msg_t sdo_msg; 

    switch (function_code)
    {
    case 3:  //TPDO1
        // memset(byte_converter.value, 0, sizeof(int)); 
        // memcpy(byte_converter.input, can_rx_msg->data, 2*sizeof(uint8_t) ); 
        
        // motor_status->status_word =  (uint16_t)byte_converter.value_u32; 

        // memcpy(byte_converter.input, &can_rx_msg->data[2], 4); 

        // motor_status->position_inc = byte_converter.value; 

       
        memcpy(&motor_status->status_word, can_rx_msg->data, 2*sizeof(uint8_t) ); 
        
        memcpy( &motor_status->position_inc, &can_rx_msg->data[2], 4); 

        // ESP_LOG_BUFFER_HEXDUMP("TPDO1", can_rx_msg->data, can_rx_msg->data_length_code, ESP_LOG_INFO);
        // ESP_LOG_BUFFER_HEXDUMP("Position", &motor_status->position_inc, 4, ESP_LOG_INFO);
        
        tpro1_flag = 1; 
        /* code */
        break;


    case 5:  //TPDO2
        // memset(byte_converter.value, 0, sizeof(int)); 
        // memcpy(byte_converter.input, can_rx_msg->data, 4 ); 
        
        // motor_status->speed_inc =  byte_converter.value; 

        // memset(byte_converter.value, 0, sizeof(int)); 
        // byte_converter.value = 0; 
        // memcpy(byte_converter.input, &can_rx_msg->data[4], 2); 

        // motor_status->actual_current = byte_converter.value; 
        
        memcpy(&motor_status->speed_inc, can_rx_msg->data, 4 ); 

        memcpy(&motor_status->actual_current, &can_rx_msg->data[4], 2); 
        tpro2_flag = 1; 

        // ESP_LOG_BUFFER_HEXDUMP("Position2", &motor_status->position_inc, 4, ESP_LOG_INFO);

        /* code */
        break;

     case 7:  //TPDO3
        // memset(byte_converter.value, 0, sizeof(int)); 
        // memcpy(byte_converter.input, can_rx_msg->data, 2 ); 
        
        // motor_status->error_code =  byte_converter.value_u32; 
        motor_status->control_mode = can_rx_msg->data[2];

        memcpy(&motor_status->error_code, can_rx_msg->data, 2 ); 
        // memset(byte_converter.value, 0, sizeof(int)); 
        // memcpy(byte_converter.input, &can_rx_msg->data[3], 1); 


        // motor_status->motor_ls = (can_rx_msg->data[3] & 1) == 0 ;
        // motor_status->centre_ls = (can_rx_msg->data[3] & (1<<1)) == 0;
        // motor_status->far_side_far_side_ls s = (can_rx_msg->data[3] & (1<<2)) == 0  ;
        motor_status->far_side_ls = (can_rx_msg->data[3] & 1) == 0 ;
        motor_status->centre_ls = (can_rx_msg->data[3] & (1<<1)) == 0;
        motor_status->motor_ls= (can_rx_msg->data[3] & (1<<2)) == 0  ;

        // ESP_LOG_BUFFER_HEXDUMP("Position3", &motor_status->position_inc, 4, ESP_LOG_INFO);

        // ESP_LOGI("LS", "Motor byte: %d", motor_status->motor_ls);
        // ESP_LOGI("LS", "centre byte: %d", motor_status->centre_ls);
        // ESP_LOGI("LS", "far byte: %d", motor_status->far_side_ls);

        // ESP_LOGI("LS", "byte: %d",can_rx_msg->data[3]);
        tpro3_flag = 1; 
            /* code */
        break;
    
  
    case 11:  //SDO
        
        

        sdo_msg.node_id = node_id; 
        
        sdo_msg.command_code = can_rx_msg->data[0]; 
        memcpy(&sdo_msg.index, &can_rx_msg->data[1],  2*sizeof(uint8_t)); 
        sdo_msg.sub_index = can_rx_msg->data[3]; 
        memcpy(sdo_msg.data.data_array, &can_rx_msg->data[4],  4*sizeof(uint8_t)); 


        xQueueSend(can_sdo_rx_queue , (void*) &sdo_msg, portMAX_DELAY);

      
        break;
    

    default:

        break;
    }

    if((tpro1_flag && tpro2_flag) && tpro3_flag)
    {

        queue_msg robot_queue_msg;
        robot_queue_msg.sender_int = 0; 
        
        memcpy(&robot_queue_msg.motor_status, motor_status, sizeof(motor_status_t)); 
        // static const char *TASK_TAG = "CAN_PROCESS";
        // ESP_LOGI(TASK_TAG, "CAN");

        // ESP_LOG_BUFFER_HEXDUMP("TPDO", &motor_status, sizeof(motor_status_t), ESP_LOG_INFO);
        xQueueSend(uart_queue, (void*) &robot_queue_msg, portMAX_DELAY); 
        tpro1_flag = 0;
        tpro2_flag = 0;
        tpro3_flag = 0; 

        // ESP_LOG_BUFFER_HEXDUMP("Position4", &robot_queue_msg.motor_status.position_inc, 4, ESP_LOG_INFO);
        // ESP_LOGI("TPDO", "Speed is %d", motor_status->speed_inc); 
    }


}

/**
 * @brief Send SDO canopen message. It also wait for an ack CAN SDO message from the motor driver. 
 *
 *
 * @param node_id: address of the received data. 
 * @param command_code: 23h write 4 bytes; 2bh write 2bytes; 2fh write 1 byte. 
 */
void sendSDO(uint8_t node_id, uint8_t command_code, uint16_t index, uint8_t sub_index,  uint32_t data_msg) {

    // union {
    //     uint8_t data[8]; 

    //     struct {
    //         uint8_t a ;  
    //         uint16_t b; 
    //         uint8_t c; 
    //         uint32_t d; 

    //     } sdo_struct; 

    // } sdo_converter = {.sdo_struct.a = command_code, .sdo_struct.b = index, .sdo_struct.c = sub_index, .sdo_struct.d = data_msg};




    twai_message_t can_message_to_send;  
    // can_message_to_send = (twai_message_t){.identifier = 0x600+node_id, .data_length_code = 8, .data = sdo_converter.data};
    can_message_to_send = (twai_message_t){.identifier = 0x600+node_id, .data_length_code = 8};

    can_message_to_send.data[0] = command_code;
    memcpy(&can_message_to_send.data[1], &index, 2*sizeof(uint8_t)); 
    can_message_to_send.data[3] = sub_index;
    memcpy(&can_message_to_send.data[4], &data_msg, 4*sizeof(uint8_t)); 
    // memcpy(can_message_to_send.data, sdo_converter.data, 8*sizeof(uint8_t));
    ESP_ERROR_CHECK(twai_transmit(&can_message_to_send , portMAX_DELAY));
    // if (twai_transmit(&can_message_to_send , portMAX_DELAY) == ESP_OK) {
    //     // printf("Message queued for transmission\n");
    // } else {
    //     printf("Failed to queue message for transmission\n");
    // }



    sdo_msg_t sdo_msg; 

    if(xQueueReceive(can_sdo_rx_queue, &sdo_msg,   100/portTICK_PERIOD_MS ) == pdPASS) // If no SDO receved in 100ms timeout. 
    {
        // printf("Ind: %x \n", sdo_msg.index); 
        
        // Todo: here we need to process SDO message if it is a read command. 

        processSDOMsg( sdo_msg);

    }
    else 
    {
        printf("SDO Timed out \n");
    }

}

/**
 * @brief 处理SDO msg, 特别是读取的信息。 
 *
 *
 * @param node_id: address of the received data. 
 * @param command_code: 23h write 4 bytes; 2bh write 2bytes; 2fh write 1 byte. 
 */

void processSDOMsg(sdo_msg_t sdo_msg)
{
    if(sdo_msg.command_code >> 4 == 4)
    {
        switch (sdo_msg.index)
        {
        case 0x6041:
            // printf("here \n"); 
            if(sdo_msg.data.data_array[0] == 0x33) // Check if the motor is enbled, if so  end the enble task. 
            {
                motor_enabled_flag = true;  
            }
            break;
        
        default:
            break;
        }
    }
}

/**
 * @brief 发送标准帧Can 
 *
 *
 * @param node_id: address of the received data. 
 * @param command_code: 23h write 4 bytes; 2bh write 2bytes; 2fh write 1 byte. 
 */
void sendGenCan(uint32_t id, uint8_t data_length, uint8_t* data_msg) {
    
    twai_message_t can_message_to_send;  
    // can_message_to_send = (twai_message_t){.identifier = 0x600+node_id, .data_length_code = 8, .data = sdo_converter.data};
    can_message_to_send = (twai_message_t){.identifier = id, .data_length_code = data_length};
    memcpy(can_message_to_send.data, data_msg, data_length*sizeof(uint8_t)); 

    ESP_ERROR_CHECK(twai_transmit(&can_message_to_send , portMAX_DELAY));
    // if (twai_transmit(&can_message_to_send , portMAX_DELAY) == ESP_OK) {
    //     printf("Message queued for transmission\n");
    // } else {
    //     printf("Failed to queue message for transmission\n");
    // }
    


}


