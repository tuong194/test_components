/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>`
#include "esp_log.h"
#include "esp_err.h"
#include "rd_ble_mesh.h"

const char *TAG = "BLE_MESH_HANDLE";

static void rd_handle_message_opcode_vender_E2(ble_mesh_cb_param_t param){
    uint8_t *buff;
    uint16_t len;
    uint32_t opcode = ble_mesh_get_opcode(param);
    ble_mesh_get_mess_buf(param, &buff, &len);
    ESP_LOGI("MESS_TYPE", "msg_size: %u, data_rec: ", len);
    for (uint8_t i = 0; i < len; i++)
    {
        printf("0x%02x ", buff[i]);
    }
    printf("\n");  
}
static void ble_mesh_sig_models_handler(void *arg, esp_event_base_t evt_base, int32_t evt_id, void *data)
{
    static bool is_first_bind = true;
    if(evt_base == EVENT_MESH_CONFIG_SERVER){
        switch (evt_id)
        {
        case EVENT_MESH_ADD_APP_KEY:
        {
            ESP_LOGI("SIG_MODEL_CB", "App key added");
            break;            
        }
        case EVENT_MESH_BIND_ALL:
        {
            ESP_LOGI("SIG_MODEL_CB", "Binding ...");            
            if(is_first_bind){
                is_first_bind = false;
            }
            break;            
        }
        case EVENT_MESH_SUB_ADD_GROUP:
        {
            ESP_LOGI("SIG_MODEL_CB", "EVENT_MESH_SUB_ADD_GROUP");
            break;            
        }
        case EVENT_MESH_SUB_DELETE_GROUP:
        {
            ESP_LOGI("SIG_MODEL_CB", "EVENT_MESH_SUB_DELETE_GROUP");
            break;            
        }
        default:
            ESP_LOGW("SIG_MODEL_CB", "Unknown config server model event id: %d", (unsigned int)evt_id);
            break;
        }
    }    
    else if(evt_base == EVENT_MESH_PROVISIONING){
        switch (evt_id)
        {
        case EVENT_MESH_PROVISION_COMPLETE:
        {
            ESP_LOGI("SIG_MODEL_CB", "Provisioning complete");
            break;            
        }
        case EVENT_MESH_PROVISION_RESET:
        {
            ESP_LOGI("SIG_MODEL_CB", "Provisioning reset, kick out now !!");
            break;            
        }
        default:
            ESP_LOGW("SIG_MODEL_CB", "Unknown prosisioning event id: %d", (unsigned int)evt_id);
            break;
        }
    }
    else if(evt_base == EVENT_MESH_GENERIC_MODEL)
    {
        switch (evt_id)
        {
        case EVENT_CONTROLL_ONOFF_BY_UNICAST_ADDR:
        {
            uint16_t data_rec = *((uint16_t *)data);
            uint8_t ele_id = (data_rec >> 8) & 0xff; 
            uint8_t onoff = data_rec & 0xff;
            ESP_LOGI("SIG_MODEL_CB", "EVENT_CONTROLL_ONOFF_BY_UNICAST_ADDR ele_id: %d, onoff: %d", ele_id, onoff);

            break;            
        }
        case EVENT_CONTROLL_ONOFF_BY_GROUP_ADDR:
        {
            uint16_t data_rec = *((uint16_t *)data);
            uint8_t ele_id = (data_rec >> 8) & 0xff; 
            uint8_t onoff = data_rec & 0xff;
            ESP_LOGI("SIG_MODEL_CB", "EVENT_CONTROLL_ONOFF_BY_GROUP_ADDR ele_id: %d, onoff: %d", ele_id, onoff);

            break;            
        }
        case EVENT_CONTROLL_ONOFF_ALL:
        {
            ESP_LOGI("SIG_MODEL_CB", "EVENT_CONTROLL_ONOFF_ALL onoff: %d", *((uint8_t *)data));

            break;            
        }
        default:
            ESP_LOGW("SIG_MODEL_CB", "Unknown generic model event id: %d", (unsigned int)evt_id);
            break;
        }
    }else if (evt_base == EVENT_MESH_SCENE_MODEL)
    {
        switch (evt_id)
        {
        uint16_t scene_id;
        case EVENT_MESH_STORE_SCENE:
        {

            ESP_LOGI("SIG_MODEL_CB", "EVENT_MESH_STORE_SCENE, scene_id");

            break;            
        }
        case EVENT_MESH_DELETE_SCENE:
        {

            ESP_LOGI("SIG_MODEL_CB", "EVENT_MESH_DELETE_SCENE, scene_id");

            break;            
        }
        case EVENT_MESH_RECALL_SCENE:
        {

            ESP_LOGI("SIG_MODEL_CB", "EVENT_MESH_RECALL_SCENE, scene_id");
            break;            
        }
        default:
            ESP_LOGW("SIG_MODEL_CB", "Unknown scene model event id: %d", (unsigned int)evt_id);
            break;
        }
    }
    
}


void app_main(void)
{
    printf("Hello world!\n");
    esp_err_t err = rd_ble_mesh_event_init();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize BLE mesh events: %s", esp_err_to_name(err));
    }
    err = rd_ble_mesh_register_event_handler(EVENT_MESH_CONFIG_SERVER, ble_mesh_sig_models_handler);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register config server event handler: %s", esp_err_to_name(err));
    }
    err = rd_ble_mesh_register_event_handler(EVENT_MESH_PROVISIONING, ble_mesh_sig_models_handler);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register provisioning event handler: %s", esp_err_to_name(err));
    }
    err = rd_ble_mesh_register_event_handler(EVENT_MESH_GENERIC_MODEL, ble_mesh_sig_models_handler);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register generic model event handler: %s", esp_err_to_name(err));
    }
    err = rd_ble_mesh_register_event_handler(EVENT_MESH_SCENE_MODEL, ble_mesh_sig_models_handler);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register scene model event handler: %s", esp_err_to_name(err));
    }
    rd_ble_mesh_register_cb_handle_mess_opcode_E2(rd_handle_message_opcode_vender_E2);
    rd_ble_mesh_init();

}
