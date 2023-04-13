#include "loop.h"

#include "esp_err.h"

QueueHandle_t uart_queue=NULL;
pc_transm_msg_t PC2mesh_msg;
pc_transm_msg_t mesh2PC_msg;

#define TAG "LOOP"
esp_err_t SLIP_prepare_packet(pc_transm_msg_t* pack);
esp_err_t SLIP_parse_packet(pc_transm_msg_t* pack);

void loop_task(void* arg) {
    const uart_port_t uart_port = UART_NUM_2;
    size_t bytes_in_buf;
    uart_event_t event;
    uart_config_t uart_cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,         /*!< UART byte size*/
        .parity = UART_PARITY_DISABLE,         /*!< UART parity mode*/
        .stop_bits = UART_STOP_BITS_1,         /*!< UART stop bits*/
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, /*!< UART HW flow control mode (cts/rts)*/
        // .rx_flow_ctrl_thresh=0;                 /*!< UART HW RTS threshold*/
        .source_clk = UART_SCLK_APB, /*!< UART source clock selection */
    };
    ESP_ERROR_CHECK(uart_driver_install(uart_port, UART_RX_BUFFER_SIZE, UART_TX_BUFFER_SIZE,10, &uart_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(uart_port, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(uart_port, 17, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_enable_pattern_det_baud_intr(uart_port, SLIP_END, 1, 9, 0, 0));
    ESP_ERROR_CHECK(uart_pattern_queue_reset(uart_port,20));
    while (1) {                
        if(xQueueReceive(uart_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
            switch((int)event.type) {
                case UART_DATA:
// warning TODO VPQ: do nothing, it is better suppress this event, but I don't now how yet;
                    break;
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG, "hw fifo overflow");
                    uart_flush_input(uart_port);
                    xQueueReset(uart_queue);
                    break;
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG, "ring buffer full");
                    uart_flush_input(uart_port);
                    xQueueReset(uart_queue);
                    break;
                case UART_BREAK:
                    ESP_LOGI(TAG, "uart rx break");
                    break;
                case UART_PARITY_ERR:
                    ESP_LOGI(TAG, "uart parity error");
                    break;
                case UART_FRAME_ERR:
                    ESP_LOGI(TAG, "uart frame error");
                    break;
                case UART_PATTERN_DET:
                    uart_get_buffered_data_len(uart_port, &bytes_in_buf);
                    int pos = uart_pattern_pop_pos(uart_port);
                    ESP_LOGI(TAG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, bytes_in_buf);
                    if (pos == -1 || pos>=bytes_in_buf) {
                        // There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
                        // record the position. We should set a larger queue size.
                        // As an example, we directly flush the rx buffer here.
                        uart_flush_input(uart_port);
                    } else {
                        pos++;
                        uart_read_bytes(uart_port, PC2mesh_msg.raw_arr, pos, 100 / portTICK_PERIOD_MS);
                        PC2mesh_msg.length=pos;
                        if(SLIP_parse_packet(&PC2mesh_msg)==ESP_OK){
                            ESP_LOGI(TAG, "receiveed length: %d", PC2mesh_msg.length);
                            ESP_LOG_BUFFER_HEX("RecData", PC2mesh_msg.raw_arr, PC2mesh_msg.length);
                            SendMessage2Node(PC2mesh_msg.ble_msg_copy_t.dst_addr ,  PC2mesh_msg.ble_msg_copy_t.opcode,
                                             PC2mesh_msg.ble_msg_copy_t.length , PC2mesh_msg.ble_msg_copy_t.msg); 
                        }
                    }
                    break;
                case NEW_MESSAGE_FROM_MESH:
                    ESP_LOGI(TAG, "new message from mesh prepared");
                    if(SLIP_prepare_packet(&mesh2PC_msg)==ESP_OK){
                        uart_write_bytes(uart_port, mesh2PC_msg.raw_arr, mesh2PC_msg.length);
                    }
                    break;
                //Others
                default:
                    ESP_LOGI(TAG, "uart event type: %d", event.type);
                    break;
            }
        }//END  if(xQueueReceive...
    }//END while(1)
}
// @brief
// @param pack
// @return
esp_err_t SLIP_prepare_packet(pc_transm_msg_t* pack) {
    uint16_t extra_length = 2;  //'END' as start and terminating symbol at transmittion
    uint16_t length;
    uint8_t* p;

    uint8_t crc=crc8(0x00,pack->raw_arr,pack->length);
    if ((pack->length + 1) > MAX_PC_MSG_LENGTH) return ESP_FAIL;  // not enough space in array for crc
    pack->raw_arr[pack->length]=crc;
    length = pack->length = pack->length + 1;
    p = pack->raw_arr;
    for (uint16_t i = 0; i < length; i++, p++) {
        if (*p == SLIP_END || *p == SLIP_ESC) {
            extra_length++;
        }
    }
    if ((length + extra_length) > MAX_PC_MSG_LENGTH) return ESP_FAIL;  // not enough space in array
    pack->length += extra_length;
    p = pack->raw_arr + length - 1;  // point to last significant byte from initial data payload
    *(p + extra_length) = SLIP_END;  // terminating 'END' symbol
    extra_length--;
    for (uint16_t i = length; i; i--) {
        switch (*p) {
            /* if it’s the same code as an END character, we send a
             * special two character code so as not to make the
             * receiver think we sent an END */
            case SLIP_END:
                *(p + extra_length) = SLIP_ESC_END;
                extra_length--;
                *(p + extra_length) = SLIP_ESC;
                break;
            /* if it’s the same code as an ESC character,
             * we send a special two character code so as not
             * to make the receiver think we sent an ESC */
            case SLIP_ESC:
                *(p + extra_length) = SLIP_ESC_ESC;
                extra_length--;
                *(p + extra_length) = SLIP_ESC;
                break;
            /* otherwise, we just send the character*/
            default:
                *(p + extra_length) = *p;
                break;
        }
        p--;
    }
    pack->raw_arr[0] = SLIP_END;  // now 'p' has to point to begining of array to transmit, i.e. 'type' member
    return ESP_OK;
}
/* RECV_PACKET: receives a packet into the buffer located at "p".
 * If more than len bytes are received, the packet will
 * be truncated.
 * Returns the number of bytes stored in the buffer.
 */
esp_err_t SLIP_parse_packet(pc_transm_msg_t* pack) {
    uint16_t length = pack->length;
    uint16_t extra_length = 0;
    uint8_t* p;

    /* sit in a loop reading bytes until we put together
     * a whole packet.
     * Make sure not to copy them into the packet if we
     * run out of room.
     */
    p = pack->raw_arr;
    uint8_t c;
    for (uint16_t i = 0; i < length; i++, p++) {
        c = *p;
        switch (c) {
            // if it’s an END character then if it is not first byte we at the end of packet
            case SLIP_END:
                /* a minor optimization: if there is no
                * data in the packet, ignore it. This is
                * meant to avoid bothering IP with all
                * the empty packets generated by the
                * duplicate END characters which are in
                Romkey [Page 5]
                RFC 1055 Serial Line IP June 1988
                * turn sent to try to detect line noise.
                */
                if (!i) {
                    extra_length++;
                    continue;
                } else if (i != length - 1) {  // somthing wrong! END must be occure only at the last data byte
                    return ESP_FAIL;
                } else {
                    pack->length -= extra_length + 1;
                    uint8_t crc=crc8(0x00,pack->raw_arr,pack->length);
                    pack->length-=1;
                    if(crc)return ESP_FAIL;
                    else return ESP_OK;
                }
                break;
            /* if it’s the same code as an ESC character, wait
             * and get another character and then figure out
             * what to store in the packet based on that.
             */
            case SLIP_ESC:
                extra_length++;
                c = *++p;
                i++;
                /* if "c" is not one of these two, then we
                 * have a protocol violation.
                 */
                switch (c) {
                    case SLIP_ESC_END:
                        *(p - extra_length) = SLIP_END;
                        break;
                    case SLIP_ESC_ESC:
                        *(p - extra_length) = SLIP_ESC;
                        break;
                    default:
                        return ESP_FAIL;
                        break;
                }
                break;
            /* here we fall into the default handler and let
             * it store the character for us
             */
            default:
                *(p - extra_length) = *p;
                break;
        }
    }
    return ESP_FAIL;  // we pass last byte and last byte is not END
}
