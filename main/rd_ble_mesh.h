#ifndef _RD_BLE_MESH_H__
#define _RD_BLE_MESH_H__

#include "stdint.h"
#include "esp_err.h"
#include "esp_event.h"

#define CID_ESP 0x0211
#define RD_BLE_MESH_MODEL_OP_1(b0)         (b0)
#define RD_BLE_MESH_MODEL_OP_2(b0, b1)     (((b0) << 8) | (b1))
#define RD_BLE_MESH_MODEL_OP_3(b0, cid)    ((((b0) << 16) | 0xC00000) | (cid))

ESP_EVENT_DECLARE_BASE(EVENT_MESH_CONFIG_SERVER);
ESP_EVENT_DECLARE_BASE(EVENT_MESH_PROVISIONING);
ESP_EVENT_DECLARE_BASE(EVENT_MESH_GENERIC_MODEL);
ESP_EVENT_DECLARE_BASE(EVENT_MESH_LIGHTING_MODEL);
ESP_EVENT_DECLARE_BASE(EVENT_MESH_SCENE_MODEL);

#define CONFIG_MAX_NUM_ELEMENT  3
#define CONFIG_USE_SDK_ROGO     0
#define LOG_RAW_DATA_BLE_ADV    0

/*!< Generic OnOff Message Opcode */
#define BLE_MESH_MODEL_OP_GEN_ONOFF_GET                         RD_BLE_MESH_MODEL_OP_2(0x82, 0x01)
#define BLE_MESH_MODEL_OP_GEN_ONOFF_SET                         RD_BLE_MESH_MODEL_OP_2(0x82, 0x02)
#define BLE_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK                   RD_BLE_MESH_MODEL_OP_2(0x82, 0x03)
#define BLE_MESH_MODEL_OP_GEN_ONOFF_STATUS                      RD_BLE_MESH_MODEL_OP_2(0x82, 0x04)

/*!< Scene Message Opcode */
#define BLE_MESH_MODEL_OP_SCENE_GET                             RD_BLE_MESH_MODEL_OP_2(0x82, 0x41)
#define BLE_MESH_MODEL_OP_SCENE_RECALL                          RD_BLE_MESH_MODEL_OP_2(0x82, 0x42)
#define BLE_MESH_MODEL_OP_SCENE_RECALL_UNACK                    RD_BLE_MESH_MODEL_OP_2(0x82, 0x43)
#define BLE_MESH_MODEL_OP_SCENE_STATUS                          RD_BLE_MESH_MODEL_OP_2(0x5E)
#define BLE_MESH_MODEL_OP_SCENE_REGISTER_GET                    RD_BLE_MESH_MODEL_OP_2(0x82, 0x44)
#define BLE_MESH_MODEL_OP_SCENE_REGISTER_STATUS                 RD_BLE_MESH_MODEL_OP_2(0x82, 0x45)

/*!< Light Lightness Message Opcode */
#define BLE_MESH_MODEL_OP_LIGHT_LIGHTNESS_GET                   RD_BLE_MESH_MODEL_OP_2(0x82, 0x4B)
#define BLE_MESH_MODEL_OP_LIGHT_LIGHTNESS_SET                   RD_BLE_MESH_MODEL_OP_2(0x82, 0x4C)
#define BLE_MESH_MODEL_OP_LIGHT_LIGHTNESS_SET_UNACK             RD_BLE_MESH_MODEL_OP_2(0x82, 0x4D)
#define BLE_MESH_MODEL_OP_LIGHT_LIGHTNESS_STATUS                RD_BLE_MESH_MODEL_OP_2(0x82, 0x4E)
#define BLE_MESH_MODEL_OP_LIGHT_LIGHTNESS_LINEAR_GET            RD_BLE_MESH_MODEL_OP_2(0x82, 0x4F)
#define BLE_MESH_MODEL_OP_LIGHT_LIGHTNESS_LINEAR_SET            RD_BLE_MESH_MODEL_OP_2(0x82, 0x50)
#define BLE_MESH_MODEL_OP_LIGHT_LIGHTNESS_LINEAR_SET_UNACK      RD_BLE_MESH_MODEL_OP_2(0x82, 0x51)
#define BLE_MESH_MODEL_OP_LIGHT_LIGHTNESS_LINEAR_STATUS         RD_BLE_MESH_MODEL_OP_2(0x82, 0x52)
#define BLE_MESH_MODEL_OP_LIGHT_LIGHTNESS_LAST_GET              RD_BLE_MESH_MODEL_OP_2(0x82, 0x53)
#define BLE_MESH_MODEL_OP_LIGHT_LIGHTNESS_LAST_STATUS           RD_BLE_MESH_MODEL_OP_2(0x82, 0x54)
#define BLE_MESH_MODEL_OP_LIGHT_LIGHTNESS_DEFAULT_GET           RD_BLE_MESH_MODEL_OP_2(0x82, 0x55)
#define BLE_MESH_MODEL_OP_LIGHT_LIGHTNESS_DEFAULT_STATUS        RD_BLE_MESH_MODEL_OP_2(0x82, 0x56)
#define BLE_MESH_MODEL_OP_LIGHT_LIGHTNESS_RANGE_GET             RD_BLE_MESH_MODEL_OP_2(0x82, 0x57)
#define BLE_MESH_MODEL_OP_LIGHT_LIGHTNESS_RANGE_STATUS          RD_BLE_MESH_MODEL_OP_2(0x82, 0x58)

/*!< Sensor Message Opcode */
#define BLE_MESH_MODEL_OP_SENSOR_STATUS                         RD_BLE_MESH_MODEL_OP_1(0x52)

typedef void *ble_mesh_cb_param_t; //ble_mesh_cb_param_t ~ (void *)
typedef void (*rd_handle_message_opcode_vender)(ble_mesh_cb_param_t param);
typedef void (*rd_ble_mesh_send_ble_adv)(int8_t rssi, uint8_t mac[6], uint8_t *buf, uint8_t len);

typedef enum {
    EVENT_MESH_ADD_APP_KEY = 0,
    EVENT_MESH_BIND_ALL,
    EVENT_MESH_SUB_ADD_GROUP,
    EVENT_MESH_SUB_DELETE_GROUP
} rd_mesh_cfg_server_event_t;

typedef enum {
    EVENT_MESH_PROVISION_COMPLETE = 0,
    EVENT_MESH_PROVISION_RESET
} rd_mesh_provisioning_event_t;

typedef enum {
    EVENT_CONTROLL_ONOFF_BY_UNICAST_ADDR = 0,
    EVENT_CONTROLL_ONOFF_BY_GROUP_ADDR,
    EVENT_CONTROLL_ONOFF_ALL
} rd_mesh_generic_model_event_t;

typedef enum {
    EVENT_CONTROLL_LIGHTNESS = 0,
    EVENT_CONTROLL_CTL_TEMPERATURE
} rd_mesh_lighting_model_event_t;

typedef enum {
    EVENT_MESH_STORE_SCENE = 0,
    EVENT_MESH_RECALL_SCENE,
    EVENT_MESH_DELETE_SCENE
} rd_mesh_scene_model_event_t;


void      rd_ble_mesh_init(void);
void      rd_continue_ble_mesh(void);
void      rd_suspend_ble_mesh(void);
esp_err_t ble_mesh_kick_out(void);
bool      ble_mesh_is_provisioned(void);

esp_err_t rd_ble_mesh_event_init(void);
esp_err_t rd_ble_mesh_register_event_handler(esp_event_base_t evt_base, esp_event_handler_t event_handler);
void      rd_ble_mesh_register_cb_handle_mess_opcode_E0(rd_handle_message_opcode_vender cb);
void      rd_ble_mesh_register_cb_handle_mess_opcode_E2(rd_handle_message_opcode_vender cb);
void      rd_ble_mesh_register_cb_send_ble_adv(rd_ble_mesh_send_ble_adv cb);

void      ble_mesh_add_group(uint16_t id_group);
void      ble_mesh_del_group(uint16_t id_group);
void      ble_mesh_get_mac(uint8_t mac[6]);
void      ble_mesh_set_gw_addr(uint16_t addr);
uint16_t  ble_mesh_get_gw_addr(void);
uint16_t  ble_mesh_get_primary_addr(void);
uint8_t   ble_mesh_get_element_index(ble_mesh_cb_param_t param);
uint16_t  ble_mesh_get_src_addr(ble_mesh_cb_param_t param);
uint16_t  ble_mesh_get_dst_addr(ble_mesh_cb_param_t param);
void      ble_mesh_get_mess_buf(ble_mesh_cb_param_t param, uint8_t **buff, uint16_t *len);
uint32_t  ble_mesh_get_opcode(ble_mesh_cb_param_t param);

esp_err_t ble_mesh_client_model_scene_send_msg(uint32_t opcode, uint16_t dst_addr, uint8_t *par, uint16_t len);
esp_err_t ble_mesh_model_sensor_send_msg(uint8_t *par, uint16_t len);
esp_err_t ble_mesh_send_opcode_vender_E1(uint8_t *par, uint16_t len);
esp_err_t ble_mesh_send_opcode_vender_E3(uint8_t *par, uint16_t len);
esp_err_t ble_mesh_update_state_onoff(uint8_t eleIdx, uint8_t onoff);
esp_err_t ble_mesh_rsp_opcode_vender_E0(ble_mesh_cb_param_t param, uint8_t *par, uint8_t len);
esp_err_t ble_mesh_rsp_opcode_vender_E2(ble_mesh_cb_param_t param, uint8_t *par, uint8_t len);

#endif /* _RD_BLE_MESH_H__ */
