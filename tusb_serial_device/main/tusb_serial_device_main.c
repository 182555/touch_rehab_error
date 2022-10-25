/* BSD Socket API Example
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdint.h>
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "addr_from_stdin.h"
#include "stateMachine.h"
#include "wifiConnection.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "cdc.h"
#include "driver/twai.h"
#include "esp_check.h"
#include "can_open_comm.h"
#include "tinyusb.h"
#include "tusb_cdc_acm.h"
#include "sdkconfig.h"
#include "esp_err.h"

#if defined(CONFIG_EXAMPLE_IPV4)
#define HOST_IP_ADDR "192.168.0.183"
#define CONFIG_EXAMPLE_IPV4_ADDR1 "192.168.51.176"
#define HOST_IP_ADDR1 CONFIG_EXAMPLE_IPV4_ADDR1
#elif defined(CONFIG_EXAMPLE_IPV6)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV6_ADDR
#else
#define HOST_IP_ADDR ""
#endif

#define PORT 54321

// static const char *TAG = "UDP_RecvFrom_PC";
// static const char *TAG1 = "UDP_SendTo_PC";
// static const char *payload = "Message from ESP32 ";

static const int RX_BUF_SIZE = 512;

// static const int UART_MSG_length = 34;

// New board pcb pin 18 for TX, pin 17 for RX.   UART1 for Fpga
#define TXD_PIN (GPIO_NUM_18)
#define RXD_PIN (GPIO_NUM_17)
#define MB_UART_RTS (GPIO_NUM_16)

// Uart 2 for PC
#define PC_TXD_PIN (GPIO_NUM_36)
#define PC_RXD_PIN (GPIO_NUM_35)
// #define HANDLE_SW_PIN (GPIO_NUM_12)
// #define RETURN_SW_PIN (GPIO_NUM_10)
// #define ESTOP_PIN (GPIO_NUM_21)
// #define GPIO_SW_PIN_SEL ((1ULL<<HANDLE_SW_PIN) | (1ULL<<RETURN_SW_PIN))

#define GPIO_OUTPUT_IO_0 (GPIO_NUM_8)
#define GPIO_OUTPUT_IO_1 (GPIO_NUM_13)
#define GPIO_OUTPUT_PIN_SEL ((1ULL << GPIO_OUTPUT_IO_0) | (1ULL << GPIO_OUTPUT_IO_1))

static SemaphoreHandle_t test_sem;

static SemaphoreHandle_t send_485_sem;

// bool uart_message_received= false;

// 初始化串口通信。




// esp_err_t tinyusb_cdc_init(int itf, const tinyusb_config_cdc_t *cfg)
// {
//     ESP_LOGD(TAG, "CDC initialization...");
//     if (itf != 0) {
//         ESP_LOGE(TAG, "There is not CDC no.%d", itf);
//         return ESP_ERR_INVALID_ARG;
//     }
//     if (cfg->cdc_class == TUSB_CLASS_CDC) {
//         ESP_RETURN_ON_ERROR(tusb_cdc_comm_init(itf), TAG, "tusb_cdc_comm_init failed");
//         cdc_obj[itf]->cdc_subclass.comm_subclass = cfg->cdc_subclass.comm_subclass;
//     } else {
//         ESP_RETURN_ON_ERROR(tusb_cdc_data_init(itf), TAG, "tusb_cdc_data_init failed");
//         cdc_obj[itf]->cdc_subclass.data_subclass = cfg->cdc_subclass.data_subclass;
//     }
//     cdc_obj[itf]->usb_dev = cfg->usb_dev;
//     return ESP_OK;
// }













void init(void)
{

    // UART 1 was used to communicate with FPGA, now used for 485 with load cell.
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        // .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
        // .rx_flow_ctrl_thresh = 120,
        .source_clk = UART_SCLK_APB,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, MB_UART_RTS, UART_PIN_NO_CHANGE);

    // Echo suppression is performed by the UART peripheral when the bit UART_RS485_CONF_REG.UART_RS485TX_RX_EN is enabled.
    REG_SET_BIT(0x3FF50044, 3);
    // For 485 communication:

    // Set UART driver mode to Half Duplex
    ESP_ERROR_CHECK(uart_set_mode(UART_NUM_1, UART_MODE_RS485_HALF_DUPLEX));
    // printf("初始化成功");

    // Uart 2 is used to comm with PC.
    const uart_config_t uart_config_2 = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        // .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
        // .rx_flow_ctrl_thresh = 120,
        .source_clk = UART_SCLK_APB,
    };

    // For PC: pin35 for TX pin 36 for RX
    uart_driver_install(UART_NUM_2, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_2, &uart_config_2);
    uart_set_pin(UART_NUM_2, PC_TXD_PIN, PC_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // 定义to pc 数组的帧头帧尾
    outputs.to_pc[0] = 0x2A;
    outputs.to_pc[33] = 0x26;
    estop_pressed = 0;
}

//
int sendData(const char *logName, const char *data)
{

    // gpio_set_level(GPIO_OUTPUT_IO_0, 1);
    // const int len = strlen(data);

    const int txBytes = uart_write_bytes(UART_NUM_1, data, 12);
    // ESP_LOGI(logName, "Wrote %d bytes", txBytes);

    // gpio_set_level(GPIO_OUTPUT_IO_0, 0);
    return txBytes;
}

// TX task for Uart 1 (485)
// After boot up send 485 once.
static void tx_task(void *arg)
{
    uint8_t Tx_buf[12] = {0xFE, 0x1, 0x7, 0x0, 0x1, 0x2, 0x0, 0x1, 0xCF, 0xFC, 0xCC, 0xFF};
    static const char *TX_TASK_TAG = "TX_TASK";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
    int txBytes = 0;

    queue_msg uart_tx_struct;
    while (1)
    {
        // if( xQueueReceive( uart_tx_queue, &uart_tx_struct,( TickType_t ) 10 ) == pdPASS )
        // {
        //     // if(uart_tx_struct.q_sender == process)
        //     // {
        //         memcpy(Tx_buf, uart_tx_struct.msg_array, 14);

        //         txBytes = uart_write_bytes(UART_NUM_1, Tx_buf, 14);
        //     // }
        // }
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay for 1000ms.

        sendData(TX_TASK_TAG, &Tx_buf);

        // const int txBytes = uart_write_bytes(UART_NUM_1, Tx_buf, 12);
        // vTaskDelay(1000/ portTICK_PERIOD_MS);

        // uart_write_bytes(UART_NUM_1, Tx_buf, 14);
        xSemaphoreTake(send_485_sem, portMAX_DELAY);

        // printf("485 tx done \n");
    }
}

/**
 * @brief 接受传感器的485 信号，处理后以队列的形式发送给上位机。
 * 如果一段时间内没有收到上位机的程序则give 485_sem 让
 *
 *
 */
static void rx_task(void *arg)
{
    // uint8_t effective_message[34] = {};
    // uint8_t working_message[34] = {};
    // uint8_t tx_buffer[34]={};
    // //bool Udp_Tx=false;

    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t *data = (uint8_t *)malloc(64 + 1);
    // // uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    // static const int UART_MSG_length = 34;

    // uint8_t* effective_message = (uint8_t*) malloc(UART_MSG_length +1);
    // uint8_t* working_message = (uint8_t*) malloc(UART_MSG_length +1);

    int count = 0;

    queue_msg msg_to_send;
    // msg_to_send.q_sender = robot;
    msg_to_send.sender_int = 0;
    bool tag = false;
    // udp_send_msg udp_to_send;

    // struct robot_info the_message ;
    // the_message.speed = 0;
    // the_message.position =2;
    // the_message.current =1 ;

    // RingbufHandle_t buf_handle;
    // buf_handle = xRingbufferCreate(1028, RINGBUF_TYPE_NOSPLIT);

    bytes_int_conv int_conv;
    queue_msg queue_to_send;
    while (1)
    {
        // gpio_set_level(GPIO_OUTPUT_IO_1, 1);
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, 14, 2000 / portTICK_RATE_MS); //

        if (rxBytes == -1)
        {
            // Receive Timed Out.
            ESP_LOGE(RX_TASK_TAG, "NO 485");
            xSemaphoreGive(send_485_sem);
        }
        else
        {
            // ESP_LOGI(RX_TASK_TAG, "485 received");
            // ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG , data, 12, ESP_LOG_INFO);
            if ((data[0] == 0xFE && data[1] == 1) && data[2] == 0x50)
            {
                int_conv.input[3] = data[4];
                int_conv.input[2] = data[5];
                int_conv.input[1] = data[6];
                int_conv.input[0] = data[7];

                // ESP_LOGI(RX_TASK_TAG, "485");
                // Torque sensor value is
                queue_to_send.lc_value = int_conv.value;
                queue_to_send.sender_int = 3; // 3 for LC

                xQueueSend(uart_queue, (void *)&queue_to_send, portMAX_DELAY);
            }
            {
                // ESP_LOGI(RX_TASK_TAG, "485 format wrong");
            }
        }
    }
    // free(data);
    // free(effective_message);
    // free(working_message);
}

// 处理电脑和机器人发来的数组信息。
static void uart_process_task(void *arg)
{
    static const char *TASK_TAG = "ProcessTag";
    int rxBytes = 34;
    // struct robot_info received_info;
    // uint8_t recv_uart_msg[34] = {};
    // // uint8_t recv_udp_msg[34] = {};
    // uint8_t effective_message[34] = {};
    // uint8_t working_message[34] = {};
    queue_msg received_struct;

    // queue_msg tx_msg;
    // tx_msg.q_sender = PC;

    udp_send_msg udp_to_send_struct;
    // Set logging level
    esp_log_level_set(TASK_TAG, ESP_LOG_INFO);
    // printf("RTOS tick period is: ");
    // printf("%d", portTICK_PERIOD_MS);
    int led_toggle = 0;
    // int Speed, S, V, F;
    bool compensation_flag = false;
    int remaining_bytes = rxBytes;
    int count = 0;
    // int Temp_F=0;

    // static const int UART_MSG_length = 34;
    while (1)
    {

        if (xQueueReceive(uart_queue, &received_struct, 1000 / portTICK_RATE_MS) == pdPASS)
        {
            // printf("xQueneRecv已执行\n");
            // ESP_LOGI(TAG1, "get into compensation");

            // printf("YES!!!");
            // ESP_LOGI(TASK_TAG, "Read %d bytes", sizeof(received_struct.msg_array));
            // printf("%d", received_info.position);
            // ESP_LOG_BUFFER_HEXDUMP(TASK_TAG, received_struct.udp_recv_array, 14, ESP_LOG_INFO);
            // ESP_LOGI(TASK_TAG, "int is %d ", received_struct.sender_int);
            // if(received_struct.q_sender == robot)
            // ESP_LOGI(TASK_TAG, "Sender is %d ", received_struct.sender_int);

            if (received_struct.sender_int == 0) // Messgae from motor driver. Process!
            {

                // ESP_LOGI(TASK_TAG, "Sender is %d ", received_struct.sender_int);
                if (led_toggle == 0)
                {
                    led_toggle = 1;
                }
                else
                {
                    led_toggle = 0;
                }

                gpio_set_level(GPIO_OUTPUT_IO_0, led_toggle);

                // printf("received_struct.sender_int == 0  ");
                memcpy(&inputs.motor_data, &received_struct.motor_status, sizeof(motor_status_t));

                // ESP_LOG_BUFFER_HEXDUMP(TASK_TAG, &received_struct.motor_status.position_inc, 4, ESP_LOG_INFO);

                // printf("开始执行fsm");
                main_fsm_function();

                if (gap < 5000000 && gap > -5000000)

                {
                    memcpy(udp_to_send_struct.udp_send_array, outputs.to_pc, sizeof(outputs.to_pc));
                    // Send robot info to PC.
                    if (xQueueSend(udp_send_queue, (void *)&udp_to_send_struct, 1000 / portTICK_RATE_MS) == pdPASS)
                    {
                        // ESP_LOG_BUFFER_HEXDUMP(TASK_TAG ,pc_msg_constructor.to_pc_array , 34, ESP_LOG_INFO);
                    }
                    else
                    {
                        ESP_LOGE(TASK_TAG, "Q send failed");
                    }
                }

                // Todo: here replace the TX message with RPDO calls.
                // memcpy(tx_msg.msg_array, outputs.to_robot, 14);
                // Send the CAN info to RPDO queue.
                if (xQueueSend(can_send_queue, (void *)&outputs, 1000 / portTICK_RATE_MS) == pdPASS)
                {
                }
                else
                    ESP_LOGE(TASK_TAG, "RPDO Q send failed");
            }
            // 每次收到的udp 信息只是储存在 inputs.pc_msg 这个变量里面。
            else if (received_struct.sender_int == 1)
            {
                if (received_struct.udp_recv_array[0] == 0xAB && received_struct.udp_recv_array[1] == 0xAB)
                {
                    // memcpy(recv_udp_msg, received_struct.udp_recv_array, 14);
                    memcpy(inputs.pc_msg, received_struct.udp_recv_array, 14);
                    //
                }
            }

            else if (received_struct.sender_int == 2)
            {
                flags.rtn_event_triggered = received_struct.process_flag;
                if (flags.rtn_event_triggered)
                {
                    ESP_LOGI(TASK_TAG, "RTN event");
                }
            }

            else if (received_struct.sender_int == 3) // Get interforce reading from Queue
            {
                // ESP_LOGI(TASK_TAG, "LC data");
                inputs.inter_force_inc = received_struct.lc_value;
                // ESP_LOGI(TASK_TAG, "LC %d", inputs.inter_force_inc);
            }
        }
        else // TODO: If Queue receive timed out, notifiy the PC for the possible reason: ESTOP or Driver ERRor or other error
        {
            ESP_LOGI(TASK_TAG, "No events received. ");
        }
        //
    }
}

// udp接收
static void udp_server_task(void *pvParameters)
{
    uint8_t rx_buffer[34];
    // uint8_t compensation_flag[1];
    //  char addr_str[128];
    int addr_family = (int)pvParameters;
    int ip_protocol = 0;
    struct sockaddr_in6 dest_addr;
    queue_msg udp_to_recv;
    // udp_to_recv.q_sender = PC;
    char *UDP_RX_TAG = "UR";
    memset(udp_to_recv.udp_recv_array, 0, 14);
    while (1)
    {

        if (addr_family == AF_INET)
        {
            struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
            dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
            dest_addr_ip4->sin_family = AF_INET;
            dest_addr_ip4->sin_port = htons(PORT);
            ip_protocol = IPPROTO_IP;
        }

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0)
        {
            // ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(UDP_RX_TAG, "Socket created");

        int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0)
        {
            // ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        }
        ESP_LOGI(UDP_RX_TAG, "Socket bound, port %d", PORT);
        ESP_LOGI(UDP_RX_TAG, "Waiting for data");

        while (1)
        {

            struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
            socklen_t socklen = sizeof(source_addr);

            //接收UDP数据
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

            // Error occurred during receiving
            if (len < 0)
            {
                ESP_LOGE(UDP_RX_TAG, "recvfrom failed: errno %d", errno);
                break;
            }
            else if (len > 0)
            {
                memcpy(udp_to_recv.udp_recv_array, rx_buffer, 14);
                // udp_to_recv.q_sender = PC;
                udp_to_recv.sender_int = 1;
                // ESP_LOG_BUFFER_HEXDUMP(UDP_RX_TAG, rx_buffer, 14, ESP_LOG_INFO);
                xQueueSend(uart_queue, (void *)&udp_to_recv, portMAX_DELAY);

                // ESP_LOGI(UDP_RX_TAG, "udp recv data: %s ", rx_buffer);
                //  memset(rx_buffer,0,14);
                // ESP_LOGI(UDP_RX_TAG, "received data ");
            }
        }

        if (sock != -1)
        {
            ESP_LOGE(UDP_RX_TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}

const char *RX_TASK_TAG = "PC_RX_TASK";

// static const char *TAG = "USB_CONNECT";

//     typedef struct mbedtls_net_context
// {
//     int fd;             /**< The underlying file descriptor                 */
// }
// mbedtls_net_context;

//   typedef struct
// {
//     mbedtls_net_context *dst;
//     const char *way;
//     const char *type;
//     unsigned len;
//     unsigned char buf[MAX_MSG_SIZE];
// } packet;

// static void pc_rx_task(int itf, cdcacm_event_t *event)
void pc_rx_task(int itf, cdcacm_event_t *event)
{
    uint8_t *buf = (uint8_t *)malloc(RX_BUF_SIZE + 1);
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);

    //  int count = 0;
    queue_msg pc_rx_to_recv;
    // pc_rx_to_recv.q_sender = PC;
    pc_rx_to_recv.sender_int = 1;
    size_t rx_size = 0;
  /* read */
    esp_err_t ret = tinyusb_cdcacm_read(itf, buf, CONFIG_TINYUSB_CDC_RX_BUFSIZE, &rx_size);
    if (ret == ESP_OK)
    {
        buf[rx_size] = '\0';
        ESP_LOGI(TAG, "Got data (%d bytes): %s", rx_size, buf);
    }
    else
    {
        ESP_LOGE(TAG, "Read error11");
    }

    // size_t rx_size = 0;
    //     uint8_t* buf = (uint8_t*) malloc(RX_BUF_SIZE+1);
    tinyusb_cdcacm_write_queue(itf, buf,34);
    tinyusb_cdcacm_write_flush(itf, 0);

   
       
    //  fflush(stdout);
    // free(buf);
    // vTaskDelete(NULL);
}

// static void pc_rx_task(void *arg)
// {
//     // uint8_t pc_message[34] = {};
//     // uint8_t tx_buffer[34]={};
//     //bool Udp_Tx=false;
//     static const char *RX_TASK_TAG = "PC_RX_TASK";
//     esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO) ;
//     uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
//     // // uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
//     // static const int UART_MSG_length = 34;

//     // uint8_t* effective_message = (uint8_t*) malloc(UART_MSG_length +1);
//     // uint8_t* working_message = (uint8_t*) malloc(UART_MSG_length +1);

//     int count = 0;
//     queue_msg pc_rx_to_recv;
//     // pc_rx_to_recv.q_sender = PC;
//     pc_rx_to_recv.sender_int = 1;
//     // bool tag=false;
//     // udp_send_msg udp_to_send;

//     // struct robot_info the_message ;
//     // the_messatagge.speed = 0;
//     // the_message.position =2;
//     // the_message.current =1 ;

//     // RingbufHandle_t buf_handle;
//     // buf_handle = xRingbufferCreate(1028, RINGBUF_TYPE_NOSPLIT);
//     while (1) {
//         // Error occurred during receiving
//         const int rxBytes = uart_read_bytes(UART_NUM_2, data, 14, 1);
//         if (rxBytes < 0) {
//             ESP_LOGE(RX_TASK_TAG, "recvfrom failed: errno %d", errno);
//             break;
//         }
//         else if(rxBytes > 0)
//         {
//             memcpy(pc_rx_to_recv.udp_recv_array, data, 14);
//             // udp_to_recv.q_sender = PC;
//             // udp_to_recv.sender_int = 1;
//             // ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, 14, ESP_LOG_INFO);
//             xQueueSend(uart_queue, (void*) &pc_rx_to_recv, portMAX_DELAY);
//             // ESP_LOGI(RX_TASK_TAG, "RX PC working");
//         }

//     }
//     free(data);
//     // free(effective_message);
//     // free(working_message);
//      vTaskDelete(NULL);
// }

// udp上行

// static void udp_Esp_to_Pc(int itf, cdcacm_event_t *event)
// {
//     // size_t rx_size = 0;
//     // uint8_t* buf = (uint8_t*) malloc(RX_BUF_SIZE+1);
//     //  tinyusb_cdcacm_write_queue(itf, buf, rx_size);
//     // tinyusb_cdcacm_write_flush(itf, 0);
// }
// static void udp_Esp_to_Pc(void *pvParameters)
// {
//     //printf("进入udp_esp_to_pc进程");
//     uint8_t udp_Tx_buf[34]={};

//     // char host_ip[] = HOST_IP_ADDR;

//     // int addr_family = 0;

//     // int ip_protocol = 0;

//     char * TAG1 = "udp_send_task";

//     udp_send_msg udp_send_struct;

//     // int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
//     // if (sock < 0) {
//     //     ESP_LOGE(TAG1, "Unable to create socket: errno %d", errno);
//     //     break;
//     // }
//     // //ESP_LOGI(TAG1, "Socket created, sending to %s:%d", HOST_IP_ADDR, 3333);

//     uint8_t j = 0;

//     while (1) {
//         if(xQueueReceive(udp_send_queue,&udp_send_struct,(TickType_t)1000)==pdPASS)
//         {

//             memcpy(udp_Tx_buf, udp_send_struct.udp_send_array, 34);

//             // int err = sendto(sock, udp_Tx_buf, 34, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));

//             // if (err < 0)
//             // {
//             //     //ESP_LOGE(TAG1, "Error occurred during sending: errno %d", errno);

//             //     shutdown(sock, 0);
//             //     close(sock);
//             //     break;
//             // }'/

//             uart_write_bytes(UART_NUM_2, udp_Tx_buf, 34);

//             // ESP_LOGI(TAG1, "Message sent");
//         }

//     }
//     vTaskDelete(NULL);
// }

/*
 * This task is used to time the press the duration of the return key press.
 * If the duration of the return button press is greater than 5 seconds, trigger reset events.
 *
 */
static void rtn_button_timer_task(void *pvParameters)
{
    int rtn_sw_status;
    queue_msg msg_to_send;
    // msg_to_send.q_sender = robot;

    msg_to_send.sender_int = 2;
    msg_to_send.process_flag = 0;

    while (1)
    {
        rtn_sw_status = gpio_get_level(RETURN_SW_PIN);
        if (rtn_sw_status == 0) // Return pressed
        {
            rtn_button_press_seconds++;
            if (rtn_button_press_seconds >= 5)
            {
                //    xEventGroupSetBits(key_press_event_group,  RTN_PRESS_EVENT_BIT);
                msg_to_send.process_flag = 1;

                // printf("Return button triggered! \n ");
            }
            else
            {
                // xEventGroupClearBits(key_press_event_group,  RTN_PRESS_EVENT_BIT);
                msg_to_send.process_flag = 0;
            }
        }
        else
        {
            // xEventGroupClearBits(key_press_event_group,  RTN_PRESS_EVENT_BIT);

            msg_to_send.process_flag = 0;
        }
        xQueueSend(uart_queue, (void *)&msg_to_send, portMAX_DELAY);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

/**
 * @brief 发送RPDO到电机。
 * .
 *
 *
 */
static void canRPDOSendTask(void *pvParameters)
{
    output_wrapper output_info;
    uint8_t control_mode_pre = 0;
    uint16_t operation_mode = 0xF;
    static const char *RPDO_TASK_TAG = "RPDO_TASK";
    esp_log_level_set(RPDO_TASK_TAG, ESP_LOG_INFO);

    // typedef union  {
    //     struct {
    //         uint8_t control_mode;
    //         uint16_t op_mode;
    //         int32_t target_position;
    //     } rpdo1_st;

    //     struct {
    //         uint8_t control_mode;
    //         int32_t target_vel;
    //         int16_t target_current;
    //     } rpdo2_st;
    //     uint8_t rpdo1_arr[7];
    // } rpdo_u  ;

    // rpdo_u  rpdo_msg;

    uint8_t rpdo1_arr[7];
    uint8_t rpdo2_arr[7];
    uint16_t op_mode;

    while (1)
    {
        if (xQueueReceive(can_send_queue, &output_info, portMAX_DELAY) == pdPASS)
        {
            // if((control_mode_pre != output_info.target_motor_paras.control_mode) ||
            //     output_info.target_motor_paras.control_mode ==1)
            // {
            rpdo1_arr[0] = output_info.target_motor_paras.control_mode;
            op_mode = (output_info.target_motor_paras.control_mode == 1) ? 0x103F : 0xF;
            memcpy(&rpdo1_arr[1], &op_mode, sizeof(uint16_t));

            memcpy(&rpdo1_arr[3], &output_info.target_motor_paras.desired_position_inc, sizeof(int));

            // ESP_LOG_BUFFER_HEXDUMP("RPDO1" , rpdo1_arr, 7, ESP_LOG_INFO);

            sendGenCan(0x200 + NODE_ID, 7, rpdo1_arr);
            // }

            control_mode_pre = output_info.target_motor_paras.control_mode;

            if (output_info.target_motor_paras.control_mode != 1)
            {
                rpdo2_arr[0] = output_info.target_motor_paras.control_mode;
                memcpy(&rpdo2_arr[1], &output_info.target_motor_paras.desired_speed_inc, sizeof(int));
                memcpy(&rpdo2_arr[5], &output_info.target_motor_paras.desired_torque, sizeof(uint16_t));

                // ESP_LOG_BUFFER_HEXDUMP("RPDO2" , rpdo2_arr, 7, ESP_LOG_INFO);

                sendGenCan(0x300 + NODE_ID, 7, rpdo2_arr);

                // ESP_LOGI("RPDO","speed target id %d", output_info.target_motor_paras.desired_speed_inc);
            }
        }
    }
}

/*
 * This function is used to implement the timed task for getting information Can info from the
 * driver at 4ms interval.
 *
 */
static void timed_task_(void *arg)
{

    TickType_t xLastWakeTime;
    //  const TickType_t xFrequency = 1000/ portTICK_PERIOD_MS;

    // Initialise the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        /* code */

        xSemaphoreTake(timer_start_sem, portMAX_DELAY); // Wait for motor init process to finish before starting the timer
        printf("Starting timer \n");
        uint8_t *temp = {0};
        for (;;)
        {
            // Wait for the next cycle.
            // xSemaphoreGive(test_sem);
            // xSemaphoreGive(send_485_sem);
            // ESP_LOGI("TimerTAG", "Tick");

            vTaskDelay(4 / portTICK_PERIOD_MS);

            // vTaskDelay(500/ portTICK_PERIOD_MS );
            sendGenCan(0x80, 0, temp);
            // Perform action here.
        }
    }
}

// static void twai_receive_task(void *arg)
// {
//     while (1) {
//         rx_task_action_t action;
//         xQueueReceive(rx_task_queue, &action, portMAX_DELAY);
//         if (action == RX_RECEIVE_PING_RESP) {
//             //Listen for ping response from slave
//             while (1) {
//                 twai_message_t rx_msg;
//                 twai_receive(&rx_msg, portMAX_DELAY);
//                 if (rx_msg.identifier == ID_SLAVE_PING_RESP) {
//                     xSemaphoreGive(stop_ping_sem);
//                     xSemaphoreGive(ctrl_task_sem);
//                     break;
//                 }
//             }
//         } else if (action == RX_RECEIVE_DATA) {
//             //Receive data messages from slave
//             uint32_t data_msgs_rec = 0;
//             while (data_msgs_rec < NO_OF_DATA_MSGS) {
//                 twai_message_t rx_msg;
//                 twai_receive(&rx_msg, portMAX_DELAY);
//                 if (rx_msg.identifier == ID_SLAVE_DATA) {
//                     uint32_t data = 0;
//                     for (int i = 0; i < rx_msg.data_length_code; i++) {
//                         data |= (rx_msg.data[i] << (i * 8));
//                     }
//                     ESP_LOGI(EXAMPLE_TAG, "Received data value %d", data);
//                     data_msgs_rec ++;
//                 }
//             }
//             xSemaphoreGive(ctrl_task_sem);
//         } else if (action == RX_RECEIVE_STOP_RESP) {
//             //Listen for stop response from slave
//             while (1) {
//                 twai_message_t rx_msg;
//                 twai_receive(&rx_msg, portMAX_DELAY);
//                 if (rx_msg.identifier == ID_SLAVE_STOP_RESP) {
//                     xSemaphoreGive(ctrl_task_sem);
//                     break;
//                 }
//             }
//         } else if (action == RX_TASK_EXIT) {
//             break;
//         }
//     }
//     vTaskDelete(NULL);
// }

// twai_message_t can_message_to_send  = {.identifier = ID_MASTER_PING, .data_length_code = 8,
//                                             .data = {1, 2 , 3 , 0 ,0 ,0 ,0 ,0}};

/**
 * @brief 初始化电机驱动.
 *
 *
 */
static void driver_init_task(void *arg)
{
    // Received rx_
    twai_message_t rx_msg;

    motor_enabled_flag = false;

    while (1)

    {
        //******************************* Reset the motor driver node*****************************/
        uint8_t init_msg[4] = {0x82, NODE_ID};

        sendGenCan(0, 2, init_msg);

        memset(init_msg, 0, 2 * sizeof(u8_t));
        sendGenCan(0x700, 0, init_msg);

        //******************************* Init the freemode of the load cell amplifier*****************************/

        // TODO: To be implelmented.

        //******************************* Clear Error *****************************/
        sendSDO(1, 0x2B, 0x6040, 0, 0x80);

        /****************************** Set control mode ***********************/
        sendSDO(1, 0x23, 0x60FF, 0, 0x0); // Set target speed to 0
        sendSDO(1, 0x2F, 0x6060, 0, 0x3); // Set control mode to speed control.

        /****************************** Motor Stop Ready Enable.  ***********************/
        sendSDO(1, 0x23, 0x6081, 0, 0x855550);
        sendSDO(1, 0x2B, 0x6040, 0, 0x80);
        sendSDO(1, 0x2B, 0x6040, 0, 0x6);
        sendSDO(1, 0x2B, 0x6040, 0, 0x7);
        sendSDO(1, 0x2B, 0x6040, 0, 0xF);

        init_msg[0] = 1;
        init_msg[1] = NODE_ID;

        sendGenCan(0, 2, init_msg);

        sendGenCan(0x700 + NODE_ID, 0, init_msg);

        sendSDO(1, 0x40, 0x6041, 0, 0x0);

        // if( xSemaphoreTake( sem_motor_enabled, ( TickType_t ) 100 ) ==pdTRUE)
        if (motor_enabled_flag == true)
        {
            printf("Motor enable done");
            xSemaphoreGive(timer_start_sem);
            xSemaphoreGive(init_done_sem); // Let the main FSM proceed from powerup to enabled.

            break;
        }

        printf("Enabling failed \n");
    }

    vTaskDelete(NULL);
}

// CAn rx thread.
static void twai_receive_task(void *arg)
{
    static twai_message_t rx_msg;
    static motor_status_t motor_status;
    while (1)
    {

        twai_receive(&rx_msg, portMAX_DELAY);
        processRxMsg(&rx_msg, &motor_status);
    }

    vTaskDelete(NULL);
}

void tinyusb_cdc_line_state_changed_callback(int itf, cdcacm_event_t *event)
{
    int dtr = event->line_state_changed_data.dtr;
    int rst = event->line_state_changed_data.rts;
    ESP_LOGI(TAG, "Line state changed! dtr:%d, rst:%d", dtr, rst);
}


// esp_err_t tinyusb_cdc_init(int itf, const tinyusb_config_cdc_t *cfg)
// {
//     ESP_LOGD(TAG, "CDC initialization...");
//     if (itf != 0) {
//         ESP_LOGE(TAG, "There is not CDC no.%d", itf);
//         return ESP_ERR_INVALID_ARG;
//     }
//     if (cfg->cdc_class == TUSB_CLASS_CDC) {
//         ESP_RETURN_ON_ERROR(tusb_cdc_comm_init(itf), TAG, "tusb_cdc_comm_init failed");
//         cdc_obj[itf]->cdc_subclass.comm_subclass = cfg->cdc_subclass.comm_subclass;
//     } else {
//         ESP_RETURN_ON_ERROR(tusb_cdc_data_init(itf), TAG, "tusb_cdc_data_init failed");
//         cdc_obj[itf]->cdc_subclass.data_subclass = cfg->cdc_subclass.data_subclass;
//     }
//     cdc_obj[itf]->usb_dev = cfg->usb_dev;
//     return ESP_OK;
// }


// esp_err_t tinyusb_cdc_deinit(int itf)
// {
//     if (itf != 0) {
//         ESP_LOGE(TAG, "There is not CDC no.%d", itf);
//         return ESP_ERR_INVALID_ARG;
//     }
//     if (cdc_obj[itf]->type == TUSB_CLASS_CDC) {
//         ESP_RETURN_ON_ERROR(tusb_cdc_deinit_comm(itf), TAG, "tusb_cdc_deinit_comm failed");
//     } else if (cdc_obj[itf]->type == TUSB_CLASS_CDC_DATA) {
//         ESP_RETURN_ON_ERROR(tusb_cdc_deinit_data(itf), TAG, "tusb_cdc_deinit_data failed");
//     } else {
//         return ESP_ERR_INVALID_ARG;
//     }
//     ESP_LOGD(TAG, "De-initialized");
//     return ESP_OK;
// }
void app_main(void)
{

  
   
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    // ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Install TWAI driver
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(EXAMPLE_TAG, "Driver installed");

    //开始运行twai
    if (twai_start() == ESP_OK)
    {
        printf("Driver started\n");
    }
    else
    {
        printf("Failed to start driver\n"); // Todo: Further work needed. LED or error code.
        return;
    }

    // printf("系统开始执行…………………………………………………………");
    key_press_event_group = xEventGroupCreate();

    // wifi_init_sta();
//   tinyusb_cdc_init(itf,*cfg);
    init();     // For Uart
    led_init(); // For onboard LED.
    // udp_client();

    // zero-initialize the config structure.
    gpio_config_t io_conf = {};
    // disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    // set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    // bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    // disable pull-down mode
    io_conf.pull_down_en = 0;
    // disable pull-up mode
    io_conf.pull_up_en = 0;
    // configure GPIO with the given settings
    gpio_config(&io_conf);

    gpio_config_t io_conf2 = {
        .intr_type = GPIO_INTR_DISABLE,  //不启用gpio中断
        .mode = GPIO_MODE_INPUT,         //输入模式
        .pin_bit_mask = GPIO_SW_PIN_SEL, //设置goio，可以同时设置多个
        .pull_down_en = 0,               // 下拉
        .pull_up_en = 1,                 // 上拉
    };
    gpio_config(&io_conf2);

    ESP_LOGI(TAG, "USB initialization");
    tinyusb_config_t tusb_cfg = {}; // the configuration using default values
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    tinyusb_config_cdcacm_t amc_cfg = {
        .usb_dev = TINYUSB_USBDEV_0,
        .cdc_port = TINYUSB_CDC_ACM_0,
        .rx_unread_buf_sz = 128,
        .callback_rx = &pc_rx_task, // the first way to register a callback
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = NULL,
        .callback_line_coding_changed = NULL};

    ESP_ERROR_CHECK(tusb_cdc_acm_init(&amc_cfg));
    /* the second way to register a callback */
    ESP_ERROR_CHECK(tinyusb_cdcacm_register_callback(
        TINYUSB_CDC_ACM_0,
        CDC_EVENT_LINE_STATE_CHANGED,
        &tinyusb_cdc_line_state_changed_callback));
    ESP_LOGI(TAG, "USB initialization DONE");

    //创建队列queue
    uart_queue = xQueueCreate(10, sizeof(queue_msg));
    udp_send_queue = xQueueCreate(10, sizeof(udp_send_msg));
    // uart_tx_queue = xQueueCreate(10, sizeof(queue_msg));
    can_send_queue = xQueueCreate(10, sizeof(output_wrapper));
    can_receive_queue = xQueueCreate(10, sizeof(twai_message_t));

    motor_status_queue = xQueueCreate(10, sizeof(motor_status_t));

    can_sdo_rx_queue = xQueueCreate(10, sizeof(sdo_msg_t));

    test_sem = xSemaphoreCreateBinary();
    send_485_sem = xSemaphoreCreateBinary();
    sem_motor_enabled = xSemaphoreCreateMutex();
    init_done_sem = xSemaphoreCreateBinary();
    timer_start_sem = xSemaphoreCreateBinary();

    static const char *TAG = "debugging";
    // printf("初始化状态机……");
    init_state_machine();
    // printf("初始化状态机成功");

    tpro1_flag = 0;

    // Use CANOpen SDO commands to init motor dirver.
    xTaskCreate(driver_init_task, "init motor driver", 8192, (void *)AF_INET, configMAX_PRIORITIES - 7, NULL);

    if (uart_queue == NULL)
    {
        // Queue cannot be created.
        printf("UART_Queue Creation Failed");
    }

    if (udp_send_queue == NULL)
    {
        // Queue cannot be created.
        printf("UDP_Send_Queue Creation Failed");
    }

    // /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
    //  * Read "Establishing Wi-Fi or Ethernet Connection" section in
    //  * examples/protocols/README.md for more information about this function.
    //  */

    // //WiFi连接
    // ESP_ERROR_CHECK(example_connect());

    //创建线程，设置优先级
    xTaskCreate(rx_task, "uart_rx_task", 2048 * 2, NULL, configMAX_PRIORITIES, NULL);
    // xTaskCreatePinnedToCore(rx_task, "uart_rx_task", 2048*2, NULL, configMAX_PRIORITIES, NULL, 0);

    // xTaskCreate(udp_Esp_to_Pc, "udp_Esp_to_Pc", 8192, NULL, configMAX_PRIORITIES-2, NULL);
    // xTaskCreatePinnedToCore(udp_Esp_to_Pc, "udp_Esp_to_Pc", 8192, NULL, configMAX_PRIORITIES-2, NULL, 0);

    xTaskCreate(tx_task, "uart_tx_task", 1024 * 2, NULL, configMAX_PRIORITIES - 4, NULL);
    // xTaskCreatePinnedToCore(tx_task, "uart_tx_task", 1024*2, NULL, configMAX_PRIORITIES-4, NULL, 1);

    xTaskCreate(uart_process_task, "uart_process_task", 4096 * 2, NULL, configMAX_PRIORITIES - 3, NULL); // 主线程。
    // xTaskCreatePinnedToCore(uart_process_task, "uart_process_task", 2024*2, NULL, configMAX_PRIORITIES-3, NULL, 1);

    // xTaskCreate(udp_server_task, "udp_server_task", 4096, (void*)AF_INET, configMAX_PRIORITIES-1, NULL);
    // xTaskCreatePinnedToCore(udp_server_task, "udp_server", 4096, (void*)AF_INET, configMAX_PRIORITIES-1, NULL, 1);
    // xTaskCreate(pc_rx_task, "pc_rx_task", 4096 * 4, (void *)AF_INET, configMAX_PRIORITIES - 1, NULL);

    xTaskCreate(timed_task_, "timed_2ms_task", 2048, (void *)AF_INET, configMAX_PRIORITIES - 8, NULL);
    //  xTaskCreate(rtn_button_timer_task, "rtn_button_timer_task", 1024, NULL, 1, NULL);// Set to have the lowest priority.
    xTaskCreate(twai_receive_task, "twai_recv_task", 4096, (void *)AF_INET, configMAX_PRIORITIES - 9, NULL);

    xTaskCreate(canRPDOSendTask, "rpdo_send_task", 4096, (void *)AF_INET, configMAX_PRIORITIES - 5, NULL);
}
