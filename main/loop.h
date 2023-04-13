#ifndef LOOP_H
#define LOOP_H


#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "crc8.h"
#include "esp_log.h"
#include "main.h"

#define LOOP_STACK_SIZE 1024
#define UART_TX_BUFFER_SIZE (UART_FIFO_LEN*2)
#define UART_RX_BUFFER_SIZE (UART_FIFO_LEN*2)

#define SLIP_END     0xc0 /* indicates end of packet */
#define SLIP_ESC     0xdb /* indicates byte stuffing */
#define SLIP_ESC_END 0xdc /* ESC ESC_END means END data byte */
#define SLIP_ESC_ESC 0xdd /* ESC ESC_ESC means ESC data byte */

void loop_task(void* arg);

enum{
    PRX_BLE_MESH_COPY=1,
};

#define MAX_PC_MSG_LENGTH 128

typedef struct __attribute__ ((packed)){
    uint16_t length; //length of payload to be sent to PC (raspbery Pi). It include size of 'type' and some of the folLowing struct
    //Used for transmitter and internally while converting to SLIP type message but not transmitted itself//
    union { 
        uint8_t raw_arr[MAX_PC_MSG_LENGTH];
        struct {//semi rough struct. Until we became know 'type' actual mapped struct is unknown
            uint8_t type; //type of message to parse properly at reception
            union{
//                uint8_t arr[MAX_PC_MSG_LENGTH-1];//One position occupide by 'type' member

                struct __attribute__ ((packed)) ble_msg_copy_t_{
                    uint16_t src_addr;
                    uint16_t dst_addr;
                    uint32_t opcode;
                    uint16_t length;//length of following 'msg' payload
                    uint8_t msg[];
                }ble_msg_copy_t;
            };
        };
    };
}pc_transm_msg_t;

//we will use same event queue with one from uart. So we will make 'event' with same pattern (structure)
//to distinguish between messages from UART and from ble mesh event type have to be have different not overlapped
// numbers
typedef enum{
    NEW_MESSAGE_FROM_MESH=(UART_EVENT_MAX+1),
}ble_to_uart_event_t;

extern pc_transm_msg_t mesh2PC_msg;
extern QueueHandle_t uart_queue;

#endif