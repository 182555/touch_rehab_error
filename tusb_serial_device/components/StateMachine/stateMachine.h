/*
 * stateMachine.h
 *
 *  Created on: 21.02.2020
 *      Author: BC
 * This script is to implement the main state machine for the 1D touch rehab robot.
 * 
 */

#ifndef STATE_MACHINE_H_
#define STATE_MACHINE_H_

#include <string.h>
#include<stdio.h>
#include<stdlib.h>
#include<math.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "led_strip.h"
#include "StateStatusLED.h"
#include "wifiConnection.h"
// #include "can_open_comm.h"

#define MSG_LENGTH_UP 34

#define HANDLE_SW_PIN (GPIO_NUM_46)
#define RETURN_SW_PIN (GPIO_NUM_3)
#define ESTOP_PIN (GPIO_NUM_21)
#define GPIO_SW_PIN_SEL ((1ULL<<HANDLE_SW_PIN) | (1ULL<<RETURN_SW_PIN)|(1ULL<<ESTOP_PIN ))

#define RTN_PRESS_EVENT_BIT BIT0

QueueHandle_t uart_queue; 
QueueHandle_t udp_send_queue;
// QueueHandle_t uart_tx_queue;
QueueHandle_t can_send_queue; 
QueueHandle_t can_receive_queue; 


static SemaphoreHandle_t timer_start_sem;
static SemaphoreHandle_t init_done_sem; 



//#define GPIO_SW_PIN_SEL ((1ULL<<HANDLE_SW_PIN) | (1ULL<<RETURN_SW_PIN))

//union内存共用结构体
typedef union 
{
    /* data */
    int value;
    uint32_t value_u32; 
    uint8_t input[4]; 
} bytes_int_conv;


static bytes_int_conv converter; 


struct flag_wrapper
{
     uint8_t rtn_event_triggered; 

} flags; 

typedef struct {
   
    int speed_inc;              // 606C
    int position_inc;           // 6064

    short actual_current;       // 6078 

    uint8_t control_mode; 

    uint16_t status_word;  //6041
    
    int position_inc_offset; // position in inc after initialisation (zeroing)

    uint16_t error_code; 

    uint8_t motor_ls; 
    uint8_t far_side_ls;
    uint8_t centre_ls; 

    

} motor_status_t;

typedef struct {
    
    short desired_torque; 
    int desired_position_inc; 
    int desired_speed_inc; 

    
    uint8_t control_mode; 

    uint16_t operation_mode; 

} target_motor_para_t; 


typedef struct 
{
    uint8_t to_robot[14]; // !Not being used anymore
    target_motor_para_t target_motor_paras; 
    uint8_t to_pc[34] ;
    int return_code ; // !Not being used anymore
} output_wrapper  ;

output_wrapper outputs; 

typedef enum {
    PC,
    robot,
    process
} queue_sender;

typedef struct 
{
    /* data */
    // uint8_t  msg_array[34];
    uint8_t  udp_recv_array[14]; // Msg from PC.
    // uint8_t  process_array[34];  
    motor_status_t motor_status; 
    int lc_value; // Inc value from the torque sensor. 
    // queue_sender q_sender; 
    int sender_int; // 0: robot_rx; 1: PC_UDP_rev; 2: Rtn_button press monitor task. 3. 485 of load cell. 
    uint8_t process_flag;  // Bit0: rtn_pressed; 

} queue_msg;

struct input_wrapper  
{
    // uint8_t robot_msg[34];
    motor_status_t motor_data; 
    int inter_force_inc; 
    uint8_t pc_msg[14]; 

} inputs; 


 typedef struct{
        uint8_t frame_start;
        uint16_t binary_inputs; 
        uint16_t running_mode; 
        uint8_t node_id; 
        uint16_t status_word; 
        uint16_t error_code; 
        int32_t position_inc; 
        int32_t speed_inc; 
        int16_t current_inc; 
        uint8_t function_code; 
        uint16_t index;
        uint8_t subindex;
        uint32_t sdo_data;
        int32_t inter_force_inc; 
        uint8_t checker; 
        int8_t frame_end;  
        

    }  to_pc_struct_t; 
union 
{
    to_pc_struct_t to_pc_struct;  
    uint8_t to_pc_array[34]; 
} pc_msg_constructor; 


typedef struct 
{
    /* data */
    uint8_t  udp_send_array[34]; 
    //queue_sender q_sender; 

} udp_send_msg;


enum ret_codes { ok, e_stop, repeat, back}; // return means return to ready state.  

enum ret_codes powerUp_state(void);
enum ret_codes enabled_state(void);
enum ret_codes initialising_state(void);
enum ret_codes ready_state(void);
enum ret_codes run_state(void);
enum ret_codes estop_state(void);
enum ret_codes  wifiConfig_state(void);
enum ret_codes devMatching_state(void);


//enum state_codes { powerUp, enabled, initialising, ready, run, estop};

struct transition {
    enum state_codes src_state;
    enum ret_codes   ret_code;
    enum state_codes dst_state;
};


// Used to filter out abnormal speed e.g. a sudden change in speed. 
int Temp_speed;
int New_speed;
int gap;

int Temp_F;

int temp_position;

int linear_speed;
int linear_position;
int inter_force;  // 交互力传感器信息。


int zero_position; 



enum state_codes cur_state;
enum ret_codes (* state_fun)(void);
enum ret_codes rc;
int estop_pressed;


static void rtn_button_timer_task(void *pvParameters); 

void setSpeed(float desired_speed);
void setCurrent(short int desired_current);
void set_control_mode(uint8_t control_mode); 
bool main_fsm_function(void); 


short int getDesiredCurrentFromPC(void); 

int getDesiredPositionFromPC(void);
void setDesiredPosition(int desired_position, int position_offset); 
int getCurrentPosition(int zero_offset);
void setCurrentPosition(int position_with_offset);
void init_state_machine(void);
bool getMotorLS(void);
bool getFarLS(void);
bool getCentreLS(void);
void setCurrentSpeed(int processed_speed);

uint8_t getCompensationFlag(void); 

void processUpwardUdpMsg(void);
void setCompParaNVS(void);
void getCompParaNVS(void);
short int Compensation(int V, int F, int resistance_I);

uint8_t abnormal_detection(int V, int game_generated_current);

void setSpeedIncFromPC(void);



static int rtn_button_press_seconds  ; 


static EventGroupHandle_t key_press_event_group;

// output_wrapper powerUp_state(input_wrapper inputs);
// output_wrapper enabled_state(input_wrapper inputs);
// output_wrapper initialising_state(input_wrapper inputs);
// output_wrapper ready_state(input_wrapper inputs);
// output_wrapper run_state(input_wrapper inputs);
// output_wrapper estop_state(input_wrapper inputs);


#define EXIT_STATE end
#define ENTRY_STATE entry
#endif /* STATE_MACHINE_H_ */