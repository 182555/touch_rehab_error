#ifndef CAN_OPEN_COMM_H_
#define CAN_OPEN_COMM_H_

#include <string.h>
#include<stdio.h>
#include<stdlib.h>
#include<math.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/twai.h"
#include "stateMachine.h"

//Example Configuration
// #define PING_PERIOD_MS          250
// #define NO_OF_DATA_MSGS         50
// #define NO_OF_ITERS             3
// #define ITER_DELAY_MS           1000
#define RX_TASK_PRIO            12
#define TX_TASK_PRIO            11
#define CTRL_TSK_PRIO          7
#define TX_GPIO_NUM             9// Tx pin
#define RX_GPIO_NUM             10
#define EXAMPLE_TAG             "TWAI Master"

#define ID_MASTER_STOP_CMD      0x0A0
#define ID_MASTER_START_CMD     0x0A1
#define ID_MASTER_PING          0x0A2
#define ID_SLAVE_STOP_RESP      0x0B0
#define ID_SLAVE_DATA           0x0B1
#define ID_SLAVE_PING_RESP      0x0B2

#define NODE_ID                 1

static const twai_message_t ping_message = {.identifier = ID_MASTER_PING, .data_length_code = 0, .rtr = 0, 
                                           .ss = 1, .data = {0, 0 , 0 , 0 ,0 ,0 ,0 ,0}};


static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_NORMAL);

// static motor_status_t motor_status; 

QueueHandle_t can_sdo_rx_queue; 
QueueHandle_t motor_status_queue; 

// Define the struct type for sdo message. 
typedef struct {
    uint8_t node_id; 
    uint8_t command_code; 
    uint16_t index; 
    uint8_t sub_index;

    union {
        uint32_t data_value;
        uint8_t data_array[4]; 
    } data;
} sdo_msg_t; 

bool motor_enabled_flag ;
static SemaphoreHandle_t sem_motor_enabled; 


// These flags are used for sync for the processing task.
// Every 4ms ESP send 0x80. The driver react to the msg to send 3 TPRO msg. 
// Only if all 3 TPRO messages are processed, the receive thread will send Q to the 
// main process Thread.  Once the main processing thread is done, the process thread will send 3 RPDOs 
// to the driver. 
uint8_t tpro1_flag, tpro2_flag, tpro3_flag; 

#endif /* STATE_MACHINE_H_ */



void sendSDO(uint8_t node_id, uint8_t command_code, uint16_t index, uint8_t sub_index,  uint32_t data_msg) ;

void processRxMsg(twai_message_t *can_rx_msg, motor_status_t *motor_status);
void sendGenCan(uint32_t id, uint8_t data_length, uint8_t* data_msg) ;
void processSDOMsg(sdo_msg_t sdo_msg); 