/*
 * stateMachine.c
 *
 *  Created on: 20220402
 *      Author: BC
 */

#include "stateMachine.h"
#define BELT_DRIVEN


// #define BLINK_GPIO               48 /* GPIO 48 for ESP32-S3 built-in addressable LED */
// #define BLINK_LED_RMT_CHANNEL    0
// #define BLINK_PERIOD             1000

// static const char *TAG = "blinkStateStatus";
// static uint8_t s_led_state = 0;
// static led_strip_t *pStrip_a;


// Use GPIO 2 on Esp32 s3 for the handle button. 
bool handle_button = false; 

int TRANS_COUNT = 14; 

static int speed_array[5] = {0};
/* array and enum below must be in sync! */
enum ret_codes (* state[])(void) = { powerUp_state, enabled_state, initialising_state, 
    ready_state, run_state, estop_state, wifiConfig_state, devMatching_state};

enum state_codes cur_state = powerUp;

enum Init_stages {move_motor, move_far, motor_ls, centre_ls, initialised, centre_error} init_stages = move_motor;

int sign_osci_record = 1; 

int far_ls_position = 0;
int motor_ls_position = 0;  

uint64_t osci_record[10] = {0}; 
uint64_t abnormal_detection_ts=0; 
/* transitions from end state aren't needed */
struct transition state_transitions[] = {
    {powerUp, repeat,  powerUp},
    {powerUp, ok,  enabled},
    {enabled, repeat,   enabled},
    {enabled, ok,  initialising},
	{enabled, back,  wifiConfig},
	{wifiConfig, repeat,  wifiConfig},

    {initialising, repeat,  initialising},
    {initialising, ok,  ready},
    {ready, repeat,  ready},
    {ready, ok , run},
    {run, repeat, run},
    {run, ok, ready},
    {estop,   ok,   powerUp},
    {estop,   repeat,   estop},
    };

/*
 * 发送初始化信号并计时： 5秒之后没有初始化，reset 重新计时，
 * 发送warning（制定一个统一的错误码， warning 码。  
 * 如果收到已经使能的信号： status word 37。 进入enabled。 
 *
 */



enum ret_codes powerUp_state(void)
{



	// if status word is 0x37. The robot is enabled. 

	outputs.to_robot[2] = outputs.to_robot[2] | (1<<1); 
	outputs.to_robot[2] = outputs.to_robot[2] & (~1); // Set reset bit to 0 

	//("已上电 锁轴");
	init_stages = move_motor;

	// if(xSemaphoreTake(init_done_sem, portMAX_DELAY) == pdTRUE)
	// {
	// 	printf("PowerUP done \n");
	// 	return ok;
	// }
	// else 
	
	// {
	// 	printf("powerup problem\n");
	// 	return repeat; 
	// }

	return ok; 

	
	// 2A 后第3位是FPGA主程序状态， 1 表示 进入run 状态。 
	// if(inputs.[3] == 1)
	// {
	// 	outputs.to_robot[2] = outputs.to_robot[2] & (~(1<<1)); 
	// 	printf("move to enabled state \n");
	// 	return ok;

	// }
	// else
	// {
	// 	// printf("Status word is: %d \n", outputs.to_robot[2]); 
	// 	return repeat; 
	// }
 
}

/*
 * Keep the speed at 0 in speed control mode. 
 * Wait for the init command. 
 */
enum ret_codes enabled_state(void)
{
	// output_wrapper outputs; 
	// memcpy(outputs.to_pc, inputs.robot_msg, MSG_LENGTH); 
	
	// Bit 2 of the byte AB AB indicates start init proces.
	bool start_init = (inputs.pc_msg[2]>>2)&1;

	// printf("Byte2 is %d ", inputs.pc_msg[2]);
	// printf("Init: %d \n",start_init); 

	ESP_LOGI("enbled", "enabled");
	// Set control mode to 0. Set speed to 0.

	set_control_mode(1);
	setSpeed(0); 
	
	// 可以用handle button 或者上位机来初始化
	if(handle_button||start_init)
	{

		
		// When entering init state reset stored LS positions for LS logic
		motor_ls_position = 0; 
		far_ls_position = 0 ;


		return ok; 
		
	}
	else if (flags.rtn_event_triggered)
	{
		/* code */
		printf("Starting smart config.");
		 start_smart_config(); 
		 return back; 
	}
	
	else {
		return repeat; 
	}
}


enum ret_codes devMatching_state(void)
{
	return repeat; 
}


enum ret_codes wifiConfig_state(void)
{
	return repeat; 
}

/*
 * Move the handle to the centre point 
 * Once in the centre move to the next state. 
 */
enum ret_codes initialising_state(void)
{
	// memcpy(outputs.to_pc, inputs.robot_msg, MSG_LENGTH_UP); 

	
	// Set the speed to 40 mm/s. Here use a swith function to switch between states. 
	// 1 move towards the motor, check if the  motor limit switch is triggerd.
	// 2
	// Hard code the positio of the two limit switches.  eg. -100000 and 1000000. Do not need to update the position on the move. 

	switch(init_stages)
    {
        case move_motor:
            set_control_mode(1);
			setSpeed(40); 
			if(getMotorLS())
			{
				setSpeed(0); 
				init_stages = motor_ls;
				printf("Motor LS reached \n"); 
				return repeat;
			}
			else if(getCentreLS())
			{
				init_stages = centre_ls; 
				setSpeed(0); 
				printf("Centre LS reached 1 \n"); 
			}
			// printf("LS byte is %d\n",(inputs.robot_msg[2] >>3) ); 
            break;

        case move_far:
			set_control_mode(1); 
			setSpeed(-40); 
		//    printf("nmd");
		    if(getFarLS())
			{
				init_stages = centre_error; 
				set_control_mode(1);
				setSpeed(0);
				printf("Far side LS reached \n");  
			}
			else if(getCentreLS())
			{
				init_stages = centre_ls; 
				setSpeed(0); 
				printf("Centre LS reached 2 \n"); 
			}

			// printf("Move to far side \n"); 

            break;

        case motor_ls:
			// Motor side limit switch is reached move back. 
			setSpeed(-40);
			init_stages = move_far; 
			printf("motor ls\n"); 

            break;

        case centre_ls:

			setSpeed(0); 
			printf("Init finished\n"); 
			init_stages = initialised; 

			zero_position = getCurrentPosition(0);  // Get the centre LS position and set to zero position. 
            break;
		
		case initialised:
            set_control_mode(1);
			setSpeed(0); 
			ESP_LOGI("Init state", "Init done");
			return ok; 
            break;

		case centre_error:
             printf("Centre limit switch is not responsive.\n");
			setSpeed(40); 
			init_stages = move_motor;
			break; 

        // operator doesn't match any case constant +, -, *, /
        default:
            printf("Error! operator is not correct");
    }

	// printf("Stage: %d\n", init_stages);
	// Check once the centre limit switch is triggered, stop. 
	if(init_stages == initialised){
		printf("Finished init!"); 
		return ok; 
	}
	else 
		return repeat; 

	

}
enum ret_codes ready_state(void)
{
	
	printf("Getting to run state. "); 
	// memcpy(outputs.to_pc, inputs.robot_msg, MSG_LENGTH_UP); 
	// 看摩擦力补偿flag 是不是1. 存到
    
	

	set_control_mode(2); 
	setCurrent(0); 
	return ok; 
}

// If in active mode read desired current and add on top of 
// friction compensation. 
enum ret_codes run_state(void)
{
	uint8_t motor_control_mode = inputs.pc_msg[3] & 3; 
	// ESP_LOGI("run_state", "Mode: %d \n", inputs.pc_msg[3]);
	bool use_desired_force = true; 
	// memcpy(outputs.to_pc, inputs.robot_msg, MSG_LENGTH_UP); 
	short int desired_current= getDesiredCurrentFromPC(); 
	int position_no_offset = getCurrentPosition(0); 

	switch(motor_control_mode)
	{


	// Position control. 
	case 0:
		
		set_control_mode(0); 
		int desiredP = getDesiredPositionFromPC();
		setDesiredPosition(desiredP, zero_position);
		// ESP_LOGI("run_state", "Mode: %d \n", motor_control_mode);
		// printf("position = %d \n", linear_position);

		// printf("desired = %d \n", desiredP);
		/* code */
		break;
	case 1: // Speed control 
			// ESP_LOGI("run_state", "Mode: %d \n", motor_control_mode);
			set_control_mode(1); 
			setSpeedIncFromPC();
		break;
	case 2: // Currrent control
		// ESP_LOGI("run_state", "Mode: %d \n", motor_control_mode);

		set_control_mode(2); 
		if(inputs.pc_msg[3]&128)
		{
			
			//ESP_LOGI(TASK_TAG, "desire current : %d",desired_current);

			

			// desired_current = (desired_current >300) ? 300:desired_current;
			// desired_current = (desired_current <-300) ? -300:desired_current;

			// setCurrent(desired_current+compensating_current);

			// Todo： here to implement the limit switch logic. 

			short virtual_wall_current = 0; 
			if(motor_ls_position !=0 && position_no_offset < motor_ls_position) // Motor ls reached. 
			{
				float temp = inputs.motor_data.speed_inc;
				if(temp<0)
					temp=0;
			
				virtual_wall_current = ((float)(position_no_offset - motor_ls_position ) * 0.25- temp/15000)+80;
				if(virtual_wall_current>-80)
				virtual_wall_current=-80;
				
				

				if(virtual_wall_current < -600)
					setCurrent(-600);
				else if(virtual_wall_current > 600)
					setCurrent(600);
				else 
					setCurrent(virtual_wall_current); 
			}
			else if (far_ls_position != 0 && position_no_offset > far_ls_position)
			{
				float temp = inputs.motor_data.speed_inc;
				if(temp>0)
					temp=0;
				virtual_wall_current = (float)(position_no_offset - far_ls_position) * 0.25 - temp/15000 +80;
				if(virtual_wall_current<50)
				virtual_wall_current=50;
				if(virtual_wall_current > 600)
					setCurrent(600);
				else if(virtual_wall_current < -600)
					setCurrent(-600);
				else 
				
					setCurrent(virtual_wall_current); 
								
			}
			else{

			
				if(getCompensationFlag())
				{
					if(abnormal_detection(linear_speed,  desired_current))
					{

						setCurrent(0);
						abnormal_detection_ts = esp_timer_get_time(); 
					} 
					else
					{
						if(esp_timer_get_time() - abnormal_detection_ts > 2000000){

							short int  compensating_current = Compensation(linear_speed, inter_force, desired_current);	
							setCurrent(compensating_current); 
						}
						
					}
					
				}
				else
				{ 	
					setCurrent(0);
				
				}
			}
		
			//printf("dectected flag; \n"); 
		}


		// abnormal_detection(linear_speed,  desired_current); 

		break; 


		

		default:

		break;
	}
	return repeat; 
}

// If estop, is released to powerup state. 
enum ret_codes estop_state(void)
{
	//printf("really estop");
	return ok; // 跳到上电状态
}

// 0 for position control. 1 for Speed control; 2 for current control. 
void set_control_mode(uint8_t control_mode)
{
	// outputs.to_robot[3] = (outputs.to_robot[3] & 0B11111100) + control_mode; 

	switch (control_mode)
	{
	case 0:/* Position control  */
		/* code */
		outputs.target_motor_paras.control_mode = 1; 
		break;
	case 1:/* Speed control  */
		/* code */
		outputs.target_motor_paras.control_mode = 3; 
		break;
	case 2:/* Torque control;  */
		/* code */
		outputs.target_motor_paras.control_mode = 4; 
		break;
	default:
		break;
	}

}

void init_state_machine(void){


	// Set default message to robot as current control and 0 current. 
	static const uint8_t msg_defaults[14] = {0xAB, 0xAB, 0, 0x82, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0}; 
	
	// We are not direct copy pc message to FPGA. Hence, we will always change outputs.to_robot
	memcpy(outputs.to_robot, msg_defaults, 14); 
	bool estop_pressed =false;
	cur_state = powerUp;

	Temp_F=0;

	Temp_speed=0;
	New_speed=0;
	gap=0;
	zero_position = 0; 

	flags.rtn_event_triggered = 0; 


}


int far_ls_counter = 0;

int motor_ls_counter = 0; 
// 状态机主程序。  if return true output robot_msg to upd. 
bool main_fsm_function(void) {

	//printf("进入状态机测试…………");
	 
	state_fun = state[cur_state];
    rc = state_fun();
	// ESP_LOGI("Main_FSM", "CS: %d", cur_state);
	bool effective_robot_msg = true; 

	// converter.input[0] = inputs.robot_msg[14];
	// converter.input[1] = inputs.robot_msg[15];
	// converter.input[2] = inputs.robot_msg[16];
	// converter.input[3] = inputs.robot_msg[17];

	//这次速度数据
	New_speed = -inputs.motor_data.speed_inc;



	// 检测新收到的速度是否和旧速度有巨大差别，如果有，放弃这一次的数据。 

	gap=New_speed-Temp_speed;
	if(gap<5000000 && gap>-5000000 )
	//(gap<1638400 && gap>-1638400 )
	{
		// 数据有效， 发送数据到UDP。 
		// xQueueSend(udp_send_queue, (void*) &udp_to_send, portMAX_DELAY);

		// ESP_LOGI(TASK_TAG, "Queue Send Over 2");
		//remaining_bytes =34; 

		// 记录有效数据到下一个循环。 
		Temp_speed= New_speed;

		temp_position = getCurrentPosition(zero_position);

		int position_dif = linear_position - temp_position; 

		if((position_dif >50000 || position_dif < -50000) && cur_state == run)
		{
			effective_robot_msg = false; 
			// printf("P %d rejected\n", temp_position); 
		}
		else 
		{
			effective_robot_msg = true;		
			linear_position = temp_position; 	
		}



	}
	else 
	{
		effective_robot_msg = false; 
		//  ESP_LOGI("Main_FSM", "Speed Error");

	}
	

	// For getting limit switch position when they are first reached. 
	if(far_ls_position ==0)
	{
		if(getFarLS()){
			far_ls_counter++; 
			if(far_ls_counter == 3)
			{
				far_ls_position = getCurrentPosition(0); 
				far_ls_counter = 0; 
				ESP_LOGI("Main_FSM", "Far ls recorded.");
			}
		}
	}

	if(motor_ls_position == 0)
	{
		if(getMotorLS()){
			motor_ls_counter ++; 
			if(motor_ls_counter == 3)
			{
				motor_ls_position = getCurrentPosition(0);
				motor_ls_counter = 0 ; 
			}
		}
	}


	linear_speed = Temp_speed; 

	// Taking moving avg of the past 5 values and send it to PC. 
	setCurrentSpeed(Temp_speed);



	//原始速度数据打印
	//printf("V = %d", V);

	
	//位置数据拆分（9-12位）
	// linear_position = getCurrentPosition(zero_position);
	setCurrentPosition(-linear_position);
	//原始位置数据打印
	

	//力数据拆分（27-30位）
	// converter.input[0] =inputs.robot_msg[28];
	// converter.input[1] =inputs.robot_msg[29];
	// converter.input[2] =inputs.robot_msg[30];
	// converter.input[3] =inputs.robot_msg[31];
	inter_force = inputs.inter_force_inc; 


	//电流数据拆分（27-30位）
	// converter.input[0] =inputs.robot_msg[18];
	// converter.input[1] =inputs.robot_msg[19];
	short int I = inputs.motor_data.actual_current; 
	//printf("I,%d", I);
	//printf("\n");
    //原始电流数据打印

	if(inter_force>1000||inter_force<-1000)
	{
		inter_force=Temp_F;
	}
	else
	{

		Temp_F = converter.value;
	}

	// Here to process the message sent to the PC.
	processUpwardUdpMsg();

	// Work out the next state. 
	if(estop_pressed!=0)
	{

		

		for (int i = 0; i < TRANS_COUNT; i++) {
			if ((cur_state == state_transitions[i].src_state) ) {
					// led_state(cur_state);

				if ((rc == state_transitions[i].ret_code) ) {
					// printf("scr: %d; ", cur_state);
					// printf("rc: %d;\n", rc);
					
					cur_state = (state_transitions[i].dst_state);
					
					led_state(cur_state);
					break;
				}
			}
		}

		// printf("dst: %d;\n", cur_state);
	}
	else { 
		cur_state = estop;
		printf("Current state: estop\n"); 
		led_state(cur_state);

	}

	return effective_robot_msg; 

}


// Set speed in mm. 
void setSpeed(float desired_speed)
{

	#ifndef BELT_DRIVEN	
    	int desired_speed_inc = (int) (-desired_speed*16384);
	#else
			int desired_speed_inc = (int) (-desired_speed*16384); // The lead of beltdirven module is 40mm twice of the previous screw module. 
	#endif 
	// outputs.to_robot[8] = desired_speed_inc  & 0xFF;
	// outputs.to_robot[9] = (desired_speed_inc >> 8) & 0xFF;
	// outputs.to_robot[10] = (desired_speed_inc  >> 16) & 0xFF;
	// outputs.to_robot[11] = (desired_speed_inc >> 24) & 0xFF;
	outputs.target_motor_paras.desired_speed_inc = desired_speed_inc; 

}


void setSpeedIncFromPC(void)
{
    // int desired_speed_inc = (int) (desired_speed*8192);

	// outputs.to_robot[8] =inputs.pc_msg[8];
	// outputs.to_robot[9] = inputs.pc_msg[9];
	// outputs.to_robot[10] = inputs.pc_msg[10];
	// outputs.to_robot[11] = inputs.pc_msg[11];

	#ifndef BELT_DRIVEN	
    	memcpy(&outputs.target_motor_paras.desired_speed_inc, &inputs.pc_msg[8], 4*sizeof(uint8_t)); 
	#else
		int temp; 
		memcpy(&temp, &inputs.pc_msg[8], 4*sizeof(uint8_t)); 

		outputs.target_motor_paras.desired_speed_inc = - temp; 
	#endif 
	
	

}

void setCurrent(short int desired_current){

	// outputs.to_robot[12] = desired_current & 0xFF;
	// outputs.to_robot[13] = (desired_current >> 8)& 0xFF;	

	#ifndef BELT_DRIVEN	
    	outputs.target_motor_paras.desired_torque = desired_current;  // 丝杆驱动转化
		
	#else
		outputs.target_motor_paras.desired_torque = -desired_current; 
	#endif 
	// outputs.target_motor_paras.desired_torque = desired_current; 
}

// 
short int getDesiredCurrentFromPC(void){
	
	short int a = inputs.pc_msg[12];
	short int b = inputs.pc_msg[13]; 
	
	return (b<<8)|a; 
}

// Returns the status of the motor side limit switch. True for limit swith is reached. 
bool getMotorLS(void)
{
	// return  (bool)((inputs.robot_msg[2] >>3) &1); 
	return inputs.motor_data.motor_ls;

} 


// Returns the status of the far motor side limit switch. True for limit swith is reached. 
bool getFarLS(void)
{
	// return (bool)((inputs.robot_msg[2] >> 5) &1); 
	return inputs.motor_data.far_side_ls; 

} 

// Returns the status of the centre side limit switch. True for limit swith is reached. 
bool getCentreLS(void)
{
	// return (bool)((inputs.robot_msg[2] >>4) & 1); 
	return inputs.motor_data.centre_ls ; 	
} 


// Get compensation flag. 

uint8_t getCompensationFlag(void)
{
	return (inputs.pc_msg[2] >>7) & 1; 
}


int getCurrentPosition(int zero_offset)
{
	// converter.input[0] = inputs.robot_msg[10];
	// converter.input[1] = inputs.robot_msg[11];
	// converter.input[2] = inputs.robot_msg[12];
	// converter.input[3] = inputs.robot_msg[13];

	return inputs.motor_data.position_inc - zero_offset;
}

// This function is used to set the current position of the handle. It replaces the raw input. 
void setCurrentPosition(int position_with_offset)
{
	// converter.value = position_with_offset; 
	
	
	// inputs.robot_msg[10] = converter.input[0] ;
	// inputs.robot_msg[11] = converter.input[1] ;
	// inputs.robot_msg[12] = converter.input[2] ; 
	// inputs.robot_msg[13] = converter.input[3] ;

	inputs.motor_data.position_inc_offset =  position_with_offset * 2; 

}


/**
 * @brief get desired position from pc. 
 *
 *
 */
int getDesiredPositionFromPC(void){
	
	converter.input[0] = inputs.pc_msg[4];
	converter.input[1] = inputs.pc_msg[5];
	converter.input[2] = inputs.pc_msg[6];
	converter.input[3] = inputs.pc_msg[7];


	return converter.value; 
}


/**
 * @brief Add zeor_offset to desired_linear position. And add it to msg array to robot. 
 *
 *
 * @param[in] desired_position: the position in inc after zeroing. 
 * @param[in] position_offset: the zero offset position. 

 */
void setDesiredPosition(int desired_position, int position_offset)
{
	

	converter.value = desired_position + position_offset; 


	outputs.target_motor_paras.desired_position_inc = converter.value; 

	outputs.to_robot[4] = converter.input[0];
	outputs.to_robot[5] = converter.input[1];
	outputs.to_robot[6] = converter.input[2];
	outputs.to_robot[7] = converter.input[3];

}

// Function  from https://gist.github.com/bmccormack/d12f4bf0c96423d03f82 to 
// calculate the moving average of actual speed.
int movingAvg(int *ptrArrNumbers, int len, int nextNum, bool reset)
{
  static long Sum = 0;
  static int pos = 0;
  //check for reset request
  if(reset){
    Sum = 0;
    pos = 0;
  }
  //Subtract the oldest number from the prev sum, add the new number
  Sum = Sum - ptrArrNumbers[pos] + nextNum;
  //Assign the nextNum to the position in the array
  ptrArrNumbers[pos] = nextNum;
  //Increment position internaly
  pos++;
  if (pos >= len){
    pos = 0;
  }
  //return the average
  return Sum / len;
}
/// This function is used to set take a moving average of current speed and replace the robot uploaed speed with it. 
void setCurrentSpeed(int processed_speed)
{
	inputs.motor_data.speed_inc =  movingAvg(speed_array, 5, processed_speed, false);
	
	// inputs.robot_msg[14] = converter.input[0] ;
	// inputs.robot_msg[15] = converter.input[1] ;
	// inputs.robot_msg[16] = converter.input[2] ; 
	// inputs.robot_msg[17] = converter.input[3] ;

}
// This function is used to process msg from robot and send it to PC.
// Current implementation includes adding handle switch and return switch info. 

void processUpwardUdpMsg(void)
{


	// ESP_LOG_BUFFER_HEXDUMP("P_out", &inputs.motor_data.position_inc, 4, ESP_LOG_INFO);
	// memcpy(outputs.to_pc, inputs.robot_msg, MSG_LENGTH_UP); 
	memcpy(&outputs.to_pc[4], &inputs.motor_data.control_mode,1); 
	memcpy(&outputs.to_pc[6], &inputs.motor_data.status_word,2); 
	memcpy(&outputs.to_pc[8], &inputs.motor_data.error_code,2); 
	memcpy(&outputs.to_pc[10], &inputs.motor_data.position_inc_offset, 4); 
	converter.value =linear_speed;
	memcpy(&outputs.to_pc[14],  &converter.input, 4); 
	memcpy(&outputs.to_pc[18], &inputs.motor_data.actual_current, 2); 
	memcpy(&outputs.to_pc[28], &inputs.inter_force_inc, 4);
	// printf("Position is %d \n", inputs.motor_data.position_inc);
	int handle_sw_status = gpio_get_level(HANDLE_SW_PIN); 
	int rtn_sw_status = gpio_get_level(RETURN_SW_PIN); 
	estop_pressed = gpio_get_level(ESTOP_PIN); 

	pc_msg_constructor.to_pc_array[1] = handle_sw_status ? pc_msg_constructor.to_pc_array[1]&(~(1<<3)) : pc_msg_constructor.to_pc_array[1] |(1<<3) ;
	pc_msg_constructor.to_pc_array[1] = rtn_sw_status ? pc_msg_constructor.to_pc_array[1]&(~(1<<4)) : pc_msg_constructor.to_pc_array[1] |(1<<4) ; 	
	
	outputs.to_pc[1] = pc_msg_constructor.to_pc_array[1]; 

	// Todo: add the limit switch bits to the output array. 

	// if(handle_sw_status)
	// {
	// 		printf("%d \n", handle_sw_status); 
	// }
	
}

/*
* This function is used to read parameter values for NVS
*
*/

int damping_coeff; 

void getCompParaNVS(void)
{

	nvs_handle_t my_handle; 
    esp_err_t err = nvs_open("comp_para_storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        // printf("Done\n");

        // Read
        printf("Reading compensation parameters from NVS ... ");
        int32_t restart_counter = 0; // value will default to 0, if not set yet in NVS
        err = nvs_get_i32(my_handle, "damping_coeff", &damping_coeff);	
        switch (err) {
            case ESP_OK:
                printf("Done\n");
                printf("Restart counter = %d\n", restart_counter);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                printf("The value is not initialized yet!\n");
                break;
            default :
                printf("Error (%s) reading!\n", esp_err_to_name(err));
        }
	}

		err = nvs_commit(my_handle);
        printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

        // Close
        nvs_close(my_handle);

}

void setCompParaNVS(void)
{
	// nvs_handle_t my_handle; 
    // esp_err_t err = nvs_open("comp_para_storage", NVS_READWRITE, &my_handle);

	//  printf("Updating restart counter in NVS ... ");
	// err = nvs_set_i32(my_handle, "restart_counter", restart_counter);
	// printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

	// err = nvs_commit(my_handle);
	// printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

	// // Close
	// nvs_close(my_handle);
}
//#define device2
//摩擦力补偿计算
short int Compensation(int V, int F, int resistance_I)
{
	
	//printf(inter_force);
  //原始数据换算
	//**********************TR1 Compensation parameters****************** 
	float damping_coefficient = 0.13;
	
	float damping_2 = -0.0001; 
    //停止速度
    short int StopSpeed = 6;
    //动摩擦补偿电流
    short int Dynamic_comp_current = 45;
    short int If = 0;
    //力放大系数
    short int K = 13;
	#ifdef device2
	damping_coeff=0.25;
	K=25;
	#endif
    //期望电流
    short int Desired_current = 0;

	short int interface_cap = 8;  


	// float damping_reduced = damping_coefficient - 0.02;
  //******************************************************************************


  //**********************TR2 Compensation parameters****************** 
    // float damping_coefficient=0.13; // Original tune

	// float damping_coefficient=0.15;
	// float damping_2 = -0.00015; 
    // //停止速度
    // short int StopSpeed=6;
    // //动摩擦补偿电流
    // short int Dynamic_comp_current=55;
    // short int If=0;
    // //力放大系数
    // short int K=12;
    // //期望电流
    // short int Desired_current=0;


 	// short int interface_cap = 7; 
  //******************************************************************************

  float linear_speed=V/16384;
  
  float interaction_force=(float) F/10;


  float damping_reduced  = damping_coefficient - 0.02;


  float damping_applied =  abs(resistance_I) > 50? damping_reduced: damping_coefficient; 


  //补偿计算
  if(linear_speed > StopSpeed)
  {
    // If=Dynamic_comp_current+linear_speed*damping_coefficient;

	 If=Dynamic_comp_current + linear_speed*damping_applied  + linear_speed*linear_speed * damping_2;  // with extra added damping for oscillation compensation. 
  }

  else

  { 
    // StopSpeed=-StopSpeed;
    if(linear_speed<-StopSpeed){
      //Dynamic_comp_current=-Dynamic_comp_current;
    //   If=-Dynamic_comp_current+linear_speed*damping_coefficient;
	  If=-Dynamic_comp_current + linear_speed * damping_applied  - linear_speed*linear_speed * damping_2;
    }
    else{
		// if(inter_force > 0.12)
		// {
		// 	If=Dynamic_comp_current;
		// }
		// else if (inter_force <-0.12)
		// {
		// 	If = -Dynamic_comp_current;
		// }
		// else{
		// 	If = 0; 
		// }
      	If=linear_speed*Dynamic_comp_current/StopSpeed;       
    }
  }

	interaction_force = (interaction_force>interface_cap) ? interface_cap:interaction_force;
	interaction_force = (interaction_force<-interface_cap) ? -interface_cap:interaction_force;

	// TR2 tuning: 
	// interaction_force = (interaction_force>7) ? 7:interaction_force;
	// interaction_force = (interaction_force<-7) ? -7:interaction_force;

  //期望电流

  	resistance_I = (resistance_I >600) ? 600:resistance_I;
	resistance_I = (resistance_I <-600) ? -600:resistance_I;
    Desired_current=If+interaction_force*K + resistance_I;
    return Desired_current; 
}

/**
 * @brief 用于监测是否有异常运动出现。 
 * @details
 *
 *
 * @param node_id: address of the received data. 
 * @param command_code: 23h write 4 bytes; 2bh write 2bytes; 2fh write 1 byte. 
 * @return error code: 0: no error; 1: speed is too fast. 2: 
 */

uint8_t abnormal_detection(int V, int game_generated_current)
{
	// int64_t t2 = esp_timer_get_time();
	int vel_osci_thres = 1638400;
    int peak_thres = 3; 

	// If game generated desired current is virtually none. No need to for abnormal detection. 
	if(abs(game_generated_current < 10 ))
	{ 
		return 0;
	}
	// Speed is too fast. 
	else if (abs(V) > 8192000)
	{
		// printf("too fast");
		return 1; }
	else 
	 {
		int threhold = vel_osci_thres * sign_osci_record;
		bool exceeding = false; 
		if(threhold > 0 )
		{
			if (V > threhold)
				exceeding = true; 
		}
		else
		{
			if (V < threhold)
				exceeding = true; 
		}
	 


		if (exceeding)
		{
			sign_osci_record = -sign_osci_record;
			int64_t current_time = esp_timer_get_time();
			
			bool detected; 
			if((current_time - osci_record[0]) > 500000)
			{
				detected = false; 
				printf("%lld\n", current_time);

			
			
			}
			else 
			{
				detected = true; 

				printf("Detected\n"); 
			}

			for(int i=0; i< peak_thres-1; i++)
			{
				osci_record[i] = osci_record[i+1];	
			}
			osci_record[peak_thres-1]= current_time; 

			if(detected)
			{return 3;}
			else
				return 0; 

			

			
		}

		else 
		{
			return 0; 
		}
	 }
}		