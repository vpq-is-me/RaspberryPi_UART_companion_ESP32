/* main.c - Application main entry point */

/*
 * Copyright (c) 2017 Intel Corporation
 * Additional Copyright (c) 2018 Espressif Systems (Shanghai) PTE LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>

#include "ble_mesh_example_init.h"
#include "board.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_local_data_operation_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_bt.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "loop.h"



//NOTE proxy server name that will be shown in phone or PC new device list is located in
//C:/esp/esp-idf/components/bt/esp_ble_mesh/mesh_core/proxy_server.c in 
//strncpy(device_name, "ESP-BLE-MESH", DEVICE_NAME_SIZE);
//but better use 
// int bt_mesh_set_device_name(const char*name) 

#define TAG "EXAMPLE"

#define CID_ESP 0x02E5
#define MAX_MSG_LENGTH 8
#define PUBLISH_RETRANSMIT_COUNT 0
#define PUBLISH_RETRANSMIT_PERIOD 50 /* not use less 50 otherwise it will turn to negative value in ESP_BLE_MESH_PUBLISH_TRANSMIT macro*/

#define CREATE_GROUP_ADDR(add) ((uint16_t)0xc000 | add)
#define GROUP_SEPTIC CREATE_GROUP_ADDR(10)
#define GROUP_DOORBELL CREATE_GROUP_ADDR(11)
#define GROUP_WATERPUMP CREATE_GROUP_ADDR(12)

#define ESP_BLE_MESH_VND_MODEL_ID_CLIENT 0x0000
#define ESP_BLE_MESH_VND_MODEL_ID_SERVER 0x0001
//Note if you want 'listen' messages from different models all possible opcode numbers have to 
//be added to vnd_op[] array!!! And set minimum length to shortest message

#define WATER_PUMP_OP_SET_PARAM               ESP_BLE_MESH_MODEL_OP_3(0x00, CID_ESP)
#define WATER_PUMP_OP_GET_PARAM               ESP_BLE_MESH_MODEL_OP_3(0x01, CID_ESP)
#define WATER_PUMP_OP_STATUS_PARAM            ESP_BLE_MESH_MODEL_OP_3(0x02, CID_ESP)
#define WATER_PUMP_OP_STATUS_COUNTER            ESP_BLE_MESH_MODEL_OP_3(0x08, CID_ESP)
#define WATER_PUMP_OP_STATUS_CAPACITY           ESP_BLE_MESH_MODEL_OP_3(0x09, CID_ESP)
#define WATER_PUMP_OP_STATUS_TK_VOL             ESP_BLE_MESH_MODEL_OP_3(0x0A, CID_ESP)
#define WATER_PUMP_OP_STATUS_ALARM              ESP_BLE_MESH_MODEL_OP_3(0x0B, CID_ESP)
#define WATER_PUMP_OP_STATUS_PP_TIMEOUTS         ESP_BLE_MESH_MODEL_OP_3(0x0C, CID_ESP)
#define WATER_PUMP_OP_STATUS_CAP_AVG            ESP_BLE_MESH_MODEL_OP_3(0x0D, CID_ESP)

#define SEPTIC_OP_SET               ESP_BLE_MESH_MODEL_OP_3(0x00, CID_ESP)
#define SEPTIC_OP_GET               ESP_BLE_MESH_MODEL_OP_3(0x01, CID_ESP)
#define SEPTIC_OP_ALARM_STATUS      ESP_BLE_MESH_MODEL_OP_3(0x03, CID_ESP)
#define SEPTIC_OP_PEND_ALARM_STATUS ESP_BLE_MESH_MODEL_OP_3(0x04, CID_ESP)

static struct ctx_str_t{
    uint16_t net_idx;
    uint16_t app_idx;
    uint16_t own_addr;
}ctx_str;
//*******refering BLE MESH supporting settings**************************** 
static uint8_t dev_uuid[ESP_BLE_MESH_OCTET16_LEN] = {0x32, 0x10};
static esp_ble_mesh_cfg_srv_t config_server = {
    .relay = ESP_BLE_MESH_RELAY_ENABLED,
    .beacon = ESP_BLE_MESH_BEACON_ENABLED,
#if defined(CONFIG_BLE_MESH_FRIEND)
    .friend_state = ESP_BLE_MESH_FRIEND_ENABLED,
#else
    .friend_state = ESP_BLE_MESH_FRIEND_NOT_SUPPORTED,
#endif
#if defined(CONFIG_BLE_MESH_GATT_PROXY_SERVER)
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_ENABLED,
#else
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_NOT_SUPPORTED,
#endif
    .default_ttl = 7,
    /* 3 transmissions with 20ms interval */
    .net_transmit = ESP_BLE_MESH_TRANSMIT(3, 50),
    .relay_retransmit = ESP_BLE_MESH_TRANSMIT(3, 50),
    //vpq: Best result 4 retransmition with 50ms interval. It can be changed from android application 
};

static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
};
static esp_ble_mesh_prov_t provision = {
    .uuid = dev_uuid,
};
//*******refering my own messages supporting settings*********************
static const esp_ble_mesh_client_op_pair_t vnd_op_pair[] = {
    { WATER_PUMP_OP_SET_PARAM, WATER_PUMP_OP_STATUS_PARAM },
    { WATER_PUMP_OP_GET_PARAM, WATER_PUMP_OP_STATUS_PARAM },
};

static esp_ble_mesh_client_t vendor_client = {
    .op_pair_size = ARRAY_SIZE(vnd_op_pair),
    .op_pair = vnd_op_pair,
};
static esp_ble_mesh_model_op_t vnd_op[] = {
    ESP_BLE_MESH_MODEL_OP(WATER_PUMP_OP_SET_PARAM, 1),//because opcode actually same for different sources set minimum length to 1 here and in other opcodes
    ESP_BLE_MESH_MODEL_OP(WATER_PUMP_OP_GET_PARAM, 1),
    ESP_BLE_MESH_MODEL_OP(WATER_PUMP_OP_STATUS_PARAM, 1),
    ESP_BLE_MESH_MODEL_OP(SEPTIC_OP_ALARM_STATUS, 1),
    ESP_BLE_MESH_MODEL_OP(SEPTIC_OP_PEND_ALARM_STATUS, 1),
    ESP_BLE_MESH_MODEL_OP(WATER_PUMP_OP_STATUS_COUNTER, 4),
    ESP_BLE_MESH_MODEL_OP(WATER_PUMP_OP_STATUS_CAPACITY, 4),
    ESP_BLE_MESH_MODEL_OP(WATER_PUMP_OP_STATUS_TK_VOL, 4),
    ESP_BLE_MESH_MODEL_OP(WATER_PUMP_OP_STATUS_ALARM, 4),
    ESP_BLE_MESH_MODEL_OP(WATER_PUMP_OP_STATUS_PP_TIMEOUTS, 8),
    ESP_BLE_MESH_MODEL_OP(WATER_PUMP_OP_STATUS_CAP_AVG, 4),
    ESP_BLE_MESH_MODEL_OP_END,
};

ESP_BLE_MESH_MODEL_PUB_DEFINE(vnd_pub, (3 + MAX_MSG_LENGTH), ROLE_NODE);  // MAX_MSG_LENGTH + length(op_code)
static esp_ble_mesh_model_t vnd_models[] = {
    ESP_BLE_MESH_VENDOR_MODEL(CID_ESP, ESP_BLE_MESH_VND_MODEL_ID_SERVER,
                              vnd_op, &vnd_pub, &vendor_client),
};

//*******some extra settings*****************************************************
// Call this function after provision completed!Even after bind key
void set_publish(void) {
    vnd_pub.retransmit = ESP_BLE_MESH_PUBLISH_TRANSMIT(PUBLISH_RETRANSMIT_COUNT, PUBLISH_RETRANSMIT_PERIOD);
    vnd_pub.ttl = 0x07;
    vnd_pub.publish_addr = GROUP_DOORBELL;
    // esp_ble_mesh_model_subscribe_group_addr(vnd_models[0].element->element_addr,
    //                                         vnd_models[0].vnd.company_id,
    //                                         vnd_models[0].vnd.model_id,
    //                                         GROUP_DOORBELL);
    esp_ble_mesh_model_subscribe_group_addr(vnd_models[0].element->element_addr,
                                            vnd_models[0].vnd.company_id,
                                            vnd_models[0].vnd.model_id,
                                            GROUP_SEPTIC);
    esp_ble_mesh_model_subscribe_group_addr(vnd_models[0].element->element_addr,
                                            vnd_models[0].vnd.company_id,
                                            vnd_models[0].vnd.model_id,
                                            GROUP_WATERPUMP);
    ctx_str.app_idx=vnd_models[0].keys[0];
}

//********************************
void print_vnd_data(void) {
    printf("\n\ndata of model\n");
    printf("model_id=%d\n", (int)(vnd_models[0].model_id));
    printf("element_idx=%d\n", (int)(vnd_models[0].element_idx));
    printf("model_idx=%d\n", (int)(vnd_models[0].model_idx));
    printf("flags=%d\n", (int)(vnd_models[0].flags));

    printf("element:element_addr=%d\n", (int)(vnd_models[0].element->element_addr));
    printf("element:location=%d\n", (int)(vnd_models[0].element->location));
    printf("element:sig_model_count=%d\n", (int)(vnd_models[0].element->sig_model_count));
    printf("element:vnd_model_count=%d\n", (int)(vnd_models[0].element->vnd_model_count));
    printf("element:*sig_models=%d\n", (int)(vnd_models[0].element->sig_models));
    printf("element:*vnd_models=%d\n", (int)(vnd_models[0].element->vnd_models));

    printf("pub:*model=%d\n", (int)(vnd_models[0].pub->model));
    printf("pub:publish_addr=%d\n", (int)(vnd_models[0].pub->publish_addr));
    printf("pub:app_idx=%d\n", (int)(vnd_models[0].pub->app_idx));
    printf("pub:cred=%d\n", (int)(vnd_models[0].pub->cred));
    printf("pub:send_rel=%d\n", (int)(vnd_models[0].pub->send_rel));
    printf("pub:ttl=%d\n", (int)(vnd_models[0].pub->ttl));
    printf("pub:retransmit=%d\n", (int)(vnd_models[0].pub->retransmit));
    printf("pub:period=%d\n", (int)(vnd_models[0].pub->period));
    printf("pub:period_div=%d\n", (int)(vnd_models[0].pub->period_div));
    printf("pub:fast_period=%d\n", (int)(vnd_models[0].pub->fast_period));
    printf("pub:count=%d\n", (int)(vnd_models[0].pub->count));
    printf("pub:period_start=%d\n", (int)(vnd_models[0].pub->period_start));
    printf("pub:*msg=%d\n", (int)(vnd_models[0].pub->msg));
    printf("pub:(*)update=%d\n", (int)(vnd_models[0].pub->update));
    printf("pub:*timer=%d\n", (int)(vnd_models[0].pub->timer.work.index));
    printf("pub:dev_role=%d\n", (int)(vnd_models[0].pub->dev_role));

    printf("app keys=[");
    for (int i = 0; i < CONFIG_BLE_MESH_MODEL_KEY_COUNT; i++)
        printf("%d ", (int)(vnd_models[0].keys[i]));
    printf("]\n");

    printf("group addr =[");
    for (int i = 0; i < CONFIG_BLE_MESH_MODEL_GROUP_COUNT; i++)
        printf("%d ", (int)(vnd_models[0].groups[i]));
    printf("]\n");

    printf("*op=%d\n", (int)(vnd_models[0].op));
    printf("*cb=%d\n", (int)(vnd_models[0].cb));
    printf("*user_data=%d\n", (int)(vnd_models[0].user_data));
}
static esp_ble_mesh_elem_t elements[] = {
        ESP_BLE_MESH_ELEMENT(0, root_models, vnd_models),
};
//**************combining MESH and my own for composition ************************
static esp_ble_mesh_comp_t composition = {
    .cid = CID_ESP,
    .elements = elements,
    .element_count = ARRAY_SIZE(elements),
};


static void prov_complete(uint16_t net_idx, uint16_t addr, uint8_t flags, uint32_t iv_index) {
    ctx_str.own_addr=addr;
    ctx_str.net_idx=net_idx;
//    ctx_str.app_idx=param->model_operation.ctx->app_idx;
    ESP_LOGI(TAG, "net_idx 0x%03x, addr 0x%04x", net_idx, addr);
    ESP_LOGI(TAG, "flags 0x%02x, iv_index 0x%08x", flags, iv_index);
    board_led_toggle(LED_B);
}

static void ble_mesh_provisioning_cb(esp_ble_mesh_prov_cb_event_t event,
                                             esp_ble_mesh_prov_cb_param_t *param) {
    switch (event) {
        case ESP_BLE_MESH_PROV_REGISTER_COMP_EVT:
            ESP_LOGI(TAG, "ESP_BLE_MESH_PROV_REGISTER_COMP_EVT, err_code %d", param->prov_register_comp.err_code);
            break;
        case ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT:
            ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT, err_code %d", param->node_prov_enable_comp.err_code);
            break;
        case ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT:
            ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT, bearer %s",
                     param->node_prov_link_open.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
            break;
        case ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT:
            ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT, bearer %s",
                     param->node_prov_link_close.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
            break;
        case ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT:
            ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT");
            prov_complete(param->node_prov_complete.net_idx, param->node_prov_complete.addr,
                          param->node_prov_complete.flags, param->node_prov_complete.iv_index);
            break;
        case ESP_BLE_MESH_NODE_PROV_RESET_EVT:
            ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_RESET_EVT");
            break;
        case ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT:
            ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT, err_code %d", param->node_set_unprov_dev_name_comp.err_code);
            break;
        default:
            break;
    }
}

static void ble_mesh_config_server_cb(esp_ble_mesh_cfg_server_cb_event_t event,
                                              esp_ble_mesh_cfg_server_cb_param_t *param) {
    if (event == ESP_BLE_MESH_CFG_SERVER_STATE_CHANGE_EVT) {
        switch (param->ctx.recv_op) {
            case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD:
                ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD");
                ESP_LOGI(TAG, "net_idx 0x%04x, app_idx 0x%04x",
                         param->value.state_change.appkey_add.net_idx,
                         param->value.state_change.appkey_add.app_idx);
                ESP_LOG_BUFFER_HEX("AppKey", param->value.state_change.appkey_add.app_key, 16);
                break;
            case ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND:
                ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND");
                ESP_LOGI(TAG, "elem_addr 0x%04x, app_idx 0x%04x, cid 0x%04x, mod_id 0x%04x",
                         param->value.state_change.mod_app_bind.element_addr,
                         param->value.state_change.mod_app_bind.app_idx,
                         param->value.state_change.mod_app_bind.company_id,
                         param->value.state_change.mod_app_bind.model_id);
                set_publish();
                ctx_str.app_idx=param->value.state_change.mod_app_bind.app_idx;
                break;
            // case ESP_BLE_MESH_MODEL_OP_RELAY_SET:
            //     ESP_LOGI(TAG, "relay %d, relay retrans 0x%02x",
            //              param->value.state_change.relay_set.relay,
            //              param->value.state_change.relay_set.relay_retransmit);
            //     break;
            default:
                break;
        }
    }
}

static void ble_mesh_custom_model_cb(esp_ble_mesh_model_cb_event_t event,
                                             esp_ble_mesh_model_cb_param_t *param) {
    int length=0;
    uint8_t*p=0;
    static uart_event_t loop_event;                                            
    switch (event) {
        case ESP_BLE_MESH_MODEL_OPERATION_EVT:
            ESP_LOGI(TAG, "new node message length %d",param->model_operation.length);
            //every message pass to PC trough UART
            mesh2PC_msg.type=PRX_BLE_MESH_COPY;
            mesh2PC_msg.ble_msg_copy_t.src_addr=param->model_operation.ctx->addr;
            mesh2PC_msg.ble_msg_copy_t.dst_addr=param->model_operation.ctx->recv_dst;
            mesh2PC_msg.ble_msg_copy_t.opcode=param->model_operation.opcode;
            length=mesh2PC_msg.ble_msg_copy_t.length=param->model_operation.length;
            p=param->model_operation.msg;
            for(int i=0;i<length;i++){
                mesh2PC_msg.ble_msg_copy_t.msg[i]=p[i];
            }
            mesh2PC_msg.length=length+offsetof(pc_transm_msg_t,ble_msg_copy_t.msg[0])
                                     -offsetof(pc_transm_msg_t,raw_arr[0]);
            loop_event.type=NEW_MESSAGE_FROM_MESH; 
            xQueueSend(uart_queue,&loop_event,0);
//            ESP_LOGI(TAG, "new message length %d",mesh2PC_msg.length);
            //END every message pass to PC trough UART
            /* This device have only one dedicated opcode to deal with - SET*/
            // if (param->model_operation.opcode == ESP_BLE_MESH_VND_MODEL_OP_SET
            //    && param->model_operation.ctx->addr == GROUP_DOORBELL) {
            //     uint8_t set_com = *param->model_operation.msg;
            //     board_led_toggle(LED_B);
            //     esp_err_t err = esp_ble_mesh_model_publish(&vnd_models[0], ESP_BLE_MESH_VND_MODEL_OP_STATUS, sizeof(set_com), (uint8_t *)&set_com, ROLE_NODE);
            //     if (err) {
            //         ESP_LOGE(TAG, "Failed to send message 0x%06x", ESP_BLE_MESH_VND_MODEL_OP_STATUS);
            //     }
            // } 
            break;
        case ESP_BLE_MESH_MODEL_SEND_COMP_EVT:
            if (param->model_send_comp.err_code) {
                ESP_LOGE(TAG, "Failed to send message 0x%06x", param->model_send_comp.opcode);
                break;
            }
            ESP_LOGI(TAG, "Send 0x%06x", param->model_send_comp.opcode);
            break;
        case ESP_BLE_MESH_MODEL_PUBLISH_COMP_EVT:
            ESP_LOGI(TAG, "Complete publish message with result err=%d", param->model_publish_comp.err_code);
            break;
        case ESP_BLE_MESH_CLIENT_MODEL_RECV_PUBLISH_MSG_EVT:
            ESP_LOGI(TAG,"We receive publish message length=%d",param->client_recv_publish_msg.length);
            ESP_LOGI(TAG, "INFO-> 0x%06x, from: %04x, to: %04x, ttl:%d, rssi: %d ", 
                                                            param->client_recv_publish_msg.opcode,
                                                            param->client_recv_publish_msg.ctx->addr,
                                                            param->client_recv_publish_msg.ctx->recv_dst,
                                                            param->client_recv_publish_msg.ctx->recv_ttl,
                                                            param->client_recv_publish_msg.ctx->recv_rssi);
            mesh2PC_msg.type=PRX_BLE_MESH_COPY;
            mesh2PC_msg.ble_msg_copy_t.src_addr=param->client_recv_publish_msg.ctx->addr;
            mesh2PC_msg.ble_msg_copy_t.dst_addr=param->client_recv_publish_msg.ctx->recv_dst;
            mesh2PC_msg.ble_msg_copy_t.opcode=param->client_recv_publish_msg.opcode;
            length=mesh2PC_msg.ble_msg_copy_t.length=param->client_recv_publish_msg.length;
            p=param->client_recv_publish_msg.msg;
            for(int i=0;i<length;i++){
                mesh2PC_msg.ble_msg_copy_t.msg[i]=p[i];
            }
            mesh2PC_msg.length=length+offsetof(pc_transm_msg_t,ble_msg_copy_t.msg[0])
                                     -offsetof(pc_transm_msg_t,raw_arr[0]);
            loop_event.type=NEW_MESSAGE_FROM_MESH; 
            xQueueSend(uart_queue,&loop_event,0);
            break;
        case ESP_BLE_MESH_CLIENT_MODEL_SEND_TIMEOUT_EVT:
            ESP_LOGI(TAG,"Peer didn't answer in timeout");
            break;
        default:
            ESP_LOGI(TAG, "Unproccessed event in model CB event: %d", event);
            break;
    }
}
#define MSG_TOUT 1000
esp_err_t SendMessage2Node(uint16_t node_addr, uint32_t opcode, uint16_t length,uint8_t*msg){
    esp_ble_mesh_msg_ctx_t ctx = {0};
    esp_err_t err;

    ctx.net_idx = ctx_str.net_idx;
    ctx.app_idx = ctx_str.app_idx;
    ctx.addr = node_addr;
    ctx.send_ttl = 7;
    ctx.send_rel = false;
    ctx.model=&vnd_models[0];
    ESP_LOGI(TAG, "SEND MSG -> net_idx=%d, app_idx=%d, dst_addr= %d",ctx_str.net_idx,ctx_str.app_idx,ctx.addr);
    err=esp_ble_mesh_client_model_send_msg(&vnd_models[0],&ctx,opcode,length,msg,MSG_TOUT,true,ROLE_NODE);
    return err;
}
void vendor_publish_message(void) {
//    static uint8_t init_done = 0;
//    struct net_buf_simple *msg = vnd_models[0].pub->msg;
    uint8_t dat = 0xab;
    esp_err_t err;
    // if (!init_done) {
    //     if (msg != NULL && vnd_models[0].pub->publish_addr != ESP_BLE_MESH_ADDR_UNASSIGNED) {
    //         bt_mesh_model_msg_init(msg, ESP_BLE_MESH_VND_MODEL_OP_STATUS);
    //         init_done = 1;
    //     } else
    //         ESP_LOGI(TAG, "can't init model messager");
    // }
    err = esp_ble_mesh_model_publish(&vnd_models[0], WATER_PUMP_OP_GET_PARAM, sizeof(dat), (uint8_t *)&dat, ROLE_NODE);
    ESP_LOGI(TAG, "try to publish, err=%d", err);

}
static esp_err_t ble_mesh_init(void) {
    esp_err_t err;

    esp_ble_mesh_register_prov_callback(ble_mesh_provisioning_cb);
    esp_ble_mesh_register_config_server_callback(ble_mesh_config_server_cb);
    esp_ble_mesh_register_custom_model_callback(ble_mesh_custom_model_cb);

    err = esp_ble_mesh_init(&provision, &composition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize mesh stack");
        return err;
    }

    err = esp_ble_mesh_client_model_init(&vnd_models[0]);
    if (err) {
        ESP_LOGE(TAG, "Failed to initialize vendor client");
        return err;
    }

    err = esp_ble_mesh_node_prov_enable(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable mesh node");
        return err;
    }

    bt_mesh_set_device_name("Home_automation");
    ESP_LOGI(TAG, "BLE Mesh Node initialized");
    set_publish();
    return ESP_OK;
}

void app_main(void) {
    esp_err_t err;

    ESP_LOGI(TAG, "Initializing...");

    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    board_init();
    err = bluetooth_init();
    if (err) {
        ESP_LOGE(TAG, "esp32_bluetooth_init failed (err %d)", err);
        return;
    }

    ble_mesh_get_dev_uuid(dev_uuid);

    /* Initialize the Bluetooth Mesh Subsystem */
    err = ble_mesh_init();
    if (err) {
        ESP_LOGE(TAG, "Bluetooth mesh init failed (err %d)", err);
    }
    xTaskCreatePinnedToCore(loop_task, "stats", 4096, NULL, 2, NULL, 1/*tskNO_AFFINITY*/);
}
//bt_mesh_net_relay
