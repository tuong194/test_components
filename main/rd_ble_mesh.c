#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "esp_log.h"
#include "nvs_flash.h"

#ifdef CONFIG_BT_BLUEDROID_ENABLED
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#endif

#ifdef CONFIG_BT_NIMBLE_ENABLED
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "console/console.h"
#endif

#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_generic_model_api.h"
#include "esp_ble_mesh_local_data_operation_api.h"
#include "esp_ble_mesh_lighting_model_api.h"
#include "esp_ble_mesh_time_scene_model_api.h"
#include "esp_ble_mesh_sensor_model_api.h"
#include "esp_ble_mesh_ble_api.h" // BLE ADV

#include "rd_ble_mesh.h"


#define TAG "CORE_BLE_MESH"

#define RD_OPCODE_E0            ESP_BLE_MESH_MODEL_OP_3(0xE0, CID_ESP)
#define RD_OPCODE_RSP_E0        ESP_BLE_MESH_MODEL_OP_3(0xE1, CID_ESP)
#define RD_OPCODE_E2            ESP_BLE_MESH_MODEL_OP_3(0xE2, CID_ESP)
#define RD_OPCODE_RSP_E2        ESP_BLE_MESH_MODEL_OP_3(0xE3, CID_ESP)

#define ESP_BLE_MESH_VND_MODEL_ID_CLIENT    0x0000
#define ESP_BLE_MESH_VND_MODEL_ID_SERVER    0x0001

#define MAX_SCENE_STORE     16
#define SCENE_VALUE_MAX_LEN 8 // max 1 bytes

ESP_EVENT_DEFINE_BASE(EVENT_MESH_CONFIG_SERVER);
ESP_EVENT_DEFINE_BASE(EVENT_MESH_PROVISIONING);
ESP_EVENT_DEFINE_BASE(EVENT_MESH_GENERIC_MODEL);
ESP_EVENT_DEFINE_BASE(EVENT_MESH_LIGHTING_MODEL);
ESP_EVENT_DEFINE_BASE(EVENT_MESH_SCENE_MODEL);

static rd_handle_message_opcode_vender handle_mess_opcode_E0 = NULL;
static rd_handle_message_opcode_vender handle_mess_opcode_E2 = NULL;
static rd_ble_mesh_send_ble_adv ble_mesh_send_ble_adv = NULL;

static uint16_t GW_ADDR = 0x0001;
static esp_event_loop_handle_t ble_mesh_event_loop;
// UUID thiết bị
static uint8_t dev_uuid[16] = {0x28, 0x04};

// Khởi tạo các giá trị để cấu hình server
static esp_ble_mesh_cfg_srv_t config_server = {
    /* 3 transmissions with 20ms interval */
    .net_transmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .relay = ESP_BLE_MESH_RELAY_ENABLED,
    .relay_retransmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .beacon = ESP_BLE_MESH_BEACON_ENABLED,
#if defined(CONFIG_BLE_MESH_GATT_PROXY_SERVER)
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_ENABLED,
#else
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_NOT_SUPPORTED,
#endif
#if defined(CONFIG_BLE_MESH_FRIEND)
    .friend_state = ESP_BLE_MESH_FRIEND_ENABLED,
#else
    .friend_state = ESP_BLE_MESH_FRIEND_NOT_SUPPORTED,
#endif
    .default_ttl = 7,
};

/*==========================================================================
                               SCENE MODEL
============================================================================*/
static struct net_buf_simple scene_buffers_0[MAX_SCENE_STORE];
static uint8_t scene_data_0[MAX_SCENE_STORE][SCENE_VALUE_MAX_LEN];
static esp_ble_mesh_scene_register_t my_scenes_0[MAX_SCENE_STORE];

static esp_ble_mesh_scenes_state_t scene_state_0 = {
    .scene_count = MAX_SCENE_STORE,
    .scenes = my_scenes_0,
};

static void init_scene_values(void)
{
    for (int i = 0; i < MAX_SCENE_STORE; i++)
    {
        net_buf_simple_init_with_data(&scene_buffers_0[i], scene_data_0[i], SCENE_VALUE_MAX_LEN);
        my_scenes_0[i].scene_number = i + 0x0001;
        my_scenes_0[i].scene_type = 0;
        my_scenes_0[i].scene_value = &scene_buffers_0[i];
    }
}

/*!<--- SCENE SEVER ---*/
ESP_BLE_MESH_MODEL_PUB_DEFINE(scene_pub_0, 5 + 3, ROLE_NODE);
static esp_ble_mesh_scene_setup_srv_t scene_setup_srv = {
    .rsp_ctrl = {
        .get_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
        .set_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
    },
    .state = &scene_state_0,
};

static esp_ble_mesh_scene_srv_t scene_srv_0 = {
    .rsp_ctrl = {
        .get_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
        .set_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
    },
    .state = &scene_state_0,
};

/*!<--- SCENE CLIENT ---*/
static esp_ble_mesh_client_t scene_client;
ESP_BLE_MESH_MODEL_PUB_DEFINE(scene_cli_pub_0, 5 + 3, ROLE_NODE);


/*==========================================================================
                               SENSOR MODEL
============================================================================*/
ESP_BLE_MESH_MODEL_PUB_DEFINE(sensor_pub, 8, ROLE_NODE);
static esp_ble_mesh_sensor_state_t sensor_states;
static esp_ble_mesh_sensor_srv_t sensor_srv = {
    .rsp_ctrl = {
        .get_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP,
        .set_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP,
    },
    .state_count = 1,
    .states = &sensor_states,
};
NET_BUF_SIMPLE_DEFINE(sensor_buf, 2);  // 2 byte data
void sensor_model_init(void)
{
    net_buf_simple_reset(&sensor_buf);
    net_buf_simple_add_u8(&sensor_buf, 0x01);
    net_buf_simple_add_u8(&sensor_buf, 0x02);

    sensor_states.sensor_property_id = 0x004F; // Temperature
    sensor_states.sensor_data.format = ESP_BLE_MESH_SENSOR_DATA_FORMAT_A;
    sensor_states.sensor_data.raw_value = &sensor_buf;
}

/*==========================================================================
                        GENERIC MODEL ONOFF
============================================================================*/

/*!<--- ONOFF SERVER ---*/
ESP_BLE_MESH_MODEL_PUB_DEFINE(onoff_pub_0, 2+3, ROLE_NODE);
static esp_ble_mesh_gen_onoff_srv_t onoff_server_0 = {
    .rsp_ctrl = {
        .get_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
        .set_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
    },
};

#if CONFIG_MAX_NUM_ELEMENT >=2
ESP_BLE_MESH_MODEL_PUB_DEFINE(onoff_pub_1, 2+3, ROLE_NODE);
static esp_ble_mesh_gen_onoff_srv_t onoff_server_1 = {
    .rsp_ctrl = {
        .get_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
        .set_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
    },
};
#endif

#if CONFIG_MAX_NUM_ELEMENT >=3
ESP_BLE_MESH_MODEL_PUB_DEFINE(onoff_pub_2, 2+3, ROLE_NODE);
static esp_ble_mesh_gen_onoff_srv_t onoff_server_2 = {
    .rsp_ctrl = {
        .get_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
        .set_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
    },
};
#endif

#if CONFIG_MAX_NUM_ELEMENT >=4
ESP_BLE_MESH_MODEL_PUB_DEFINE(onoff_pub_3, 2+3, ROLE_NODE);
static esp_ble_mesh_gen_onoff_srv_t onoff_server_3 = {
    .rsp_ctrl = {
        .get_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
        .set_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP, //ESP_BLE_MESH_SERVER_RSP_BY_APP,
    },
};
#endif

#if CONFIG_BLE_MESH_GENERIC_ONOFF_CLI
/*!<--- ONOFF CLIENT ---*/
static esp_ble_mesh_client_t onoff_client_0;
ESP_BLE_MESH_MODEL_PUB_DEFINE(onoff_client_pub, 2 + 1, ROLE_NODE);

#if CONFIG_MAX_NUM_ELEMENT >=2
static esp_ble_mesh_client_t onoff_client_1;
ESP_BLE_MESH_MODEL_PUB_DEFINE(onoff_client_pub1, 2 + 1, ROLE_NODE);
#endif

#if CONFIG_MAX_NUM_ELEMENT >=3
static esp_ble_mesh_client_t onoff_client_2;
ESP_BLE_MESH_MODEL_PUB_DEFINE(onoff_client_pub2, 2 + 1, ROLE_NODE);
#endif

#if CONFIG_MAX_NUM_ELEMENT >=4
static esp_ble_mesh_client_t onoff_client_3;
ESP_BLE_MESH_MODEL_PUB_DEFINE(onoff_client_pub3, 2 + 1, ROLE_NODE);
#endif
#endif

#if CONFIG_ENABLE_LIGHT_DIM_CCT
/*==========================================================================
                            LIGHTING MODEL
============================================================================*/
static esp_ble_mesh_light_lightness_state_t lightness_state = {
    .lightness_linear          = 0x0000,
    .target_lightness_linear   = 0x0000,

    .lightness_actual          = 0x0000,
    .target_lightness_actual   = 0x0000,

    .lightness_last            = 0x0000,
    .lightness_default         = 0x0000,

    .status_code               = 0x00,     

    .lightness_range_min       = 0x0001,   
    .lightness_range_max       = 0xFFFF,   
};

ESP_BLE_MESH_MODEL_PUB_DEFINE(lightness_pub_0, 2+3, ROLE_NODE);
static esp_ble_mesh_light_lightness_srv_t lightness_server_0 = {
    .rsp_ctrl = {
        .get_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
        .set_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
    },
    .state = &lightness_state,
};

/**
 * @brief lighting model: cct
 * 
 */
esp_ble_mesh_light_ctl_state_t light_cct ={
    .lightness           = 0x0000,
    .target_lightness    = 0x0000,

    .temperature         = 0x0320,   // 800 Kelvil
    .target_temperature  = 0x0320,   // 20000 Kelvil

    .delta_uv            = 0x0000,
    .target_delta_uv     = 0x0000,

    .status_code         = 0x00,     // SUCCESS

    .temperature_range_min = 0x0320,  // >= 800
    .temperature_range_max = 0x4E20,  // <= 20000

    .lightness_default   = 0x0000,
    .temperature_default = 0x0320,
    .delta_uv_default    = 0x0000,    
};

ESP_BLE_MESH_MODEL_PUB_DEFINE(lightcct_pub_0, 2+3, ROLE_NODE);
static esp_ble_mesh_light_ctl_temp_srv_t lightcct_server_0 = {
    .rsp_ctrl = {
        .get_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
        .set_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
    },
    .state = &light_cct,
};
#endif

/**
 * @brief Cấu hình sig model cho các node
 * 
 */
static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
    ESP_BLE_MESH_MODEL_GEN_ONOFF_SRV(&onoff_pub_0, &onoff_server_0),
    ESP_BLE_MESH_MODEL_SENSOR_SRV(&sensor_pub, &sensor_srv),
#if CONFIG_BLE_MESH_GENERIC_ONOFF_CLI
    ESP_BLE_MESH_MODEL_GEN_ONOFF_CLI(&onoff_client_pub, &onoff_client_0),
#endif
#if CONFIG_ENABLE_LIGHT_DIM_CCT
    ESP_BLE_MESH_MODEL_LIGHT_LIGHTNESS_SRV(&lightness_pub_0, &lightness_server_0),
    ESP_BLE_MESH_MODEL_LIGHT_CTL_TEMP_SRV(&lightcct_pub_0, &lightcct_server_0),
#endif
    ESP_BLE_MESH_MODEL_SCENE_SRV(&scene_pub_0, &scene_srv_0),    // recall scene
    ESP_BLE_MESH_MODEL_SCENE_SETUP_SRV(NULL, &scene_setup_srv),  // store/delete scene    
    ESP_BLE_MESH_MODEL_SCENE_CLI(&scene_cli_pub_0, &scene_client),
};

#if CONFIG_MAX_NUM_ELEMENT >=2
esp_ble_mesh_model_t sig_models1[] = {
    ESP_BLE_MESH_MODEL_GEN_ONOFF_SRV(&onoff_pub_1, &onoff_server_1),
};
#endif

#if CONFIG_MAX_NUM_ELEMENT >=3
esp_ble_mesh_model_t sig_models2[] = {
    ESP_BLE_MESH_MODEL_GEN_ONOFF_SRV(&onoff_pub_2, &onoff_server_2),
};
#endif

#if CONFIG_MAX_NUM_ELEMENT >=4
esp_ble_mesh_model_t sig_models3[] = {
    ESP_BLE_MESH_MODEL_GEN_ONOFF_SRV(&onoff_pub_3, &onoff_server_3),
};
#endif

/*==========================================================================
                            VENDER MODEL
============================================================================*/
static esp_ble_mesh_model_op_t vnd_op[] = {
    ESP_BLE_MESH_MODEL_OP(RD_OPCODE_E0, 2), // RD_NOTE: config opcode vender
    ESP_BLE_MESH_MODEL_OP(RD_OPCODE_E2, 2),
    ESP_BLE_MESH_MODEL_OP_END,
};

#if CONFIG_MAX_NUM_ELEMENT >=2
static esp_ble_mesh_model_op_t vnd_op1[] = {
    ESP_BLE_MESH_MODEL_OP(RD_OPCODE_E0, 2),
    ESP_BLE_MESH_MODEL_OP(RD_OPCODE_E2, 2),
    ESP_BLE_MESH_MODEL_OP_END,
};
#endif

#if CONFIG_MAX_NUM_ELEMENT >=3
static esp_ble_mesh_model_op_t vnd_op2[] = {
    ESP_BLE_MESH_MODEL_OP(RD_OPCODE_E0, 2),
    ESP_BLE_MESH_MODEL_OP(RD_OPCODE_E2, 2),
    ESP_BLE_MESH_MODEL_OP_END,
};
#endif

#if CONFIG_MAX_NUM_ELEMENT >=4
static esp_ble_mesh_model_op_t vnd_op3[] = {
    ESP_BLE_MESH_MODEL_OP(RD_OPCODE_E0, 2),
    ESP_BLE_MESH_MODEL_OP(RD_OPCODE_E2, 2),
    ESP_BLE_MESH_MODEL_OP_END,
};
#endif

/**
 * @brief Cấu hình vender model cho các node
 * 
 */
esp_ble_mesh_model_t vnd_models[] = {
    ESP_BLE_MESH_VENDOR_MODEL(CID_ESP, ESP_BLE_MESH_VND_MODEL_ID_SERVER, vnd_op, NULL, NULL),
};

#if CONFIG_MAX_NUM_ELEMENT >=2
esp_ble_mesh_model_t vnd_models1[] = {
    ESP_BLE_MESH_VENDOR_MODEL(CID_ESP, ESP_BLE_MESH_VND_MODEL_ID_SERVER, vnd_op1, NULL, NULL),
};
#endif

#if CONFIG_MAX_NUM_ELEMENT >=3
esp_ble_mesh_model_t vnd_models2[] = {
    ESP_BLE_MESH_VENDOR_MODEL(CID_ESP, ESP_BLE_MESH_VND_MODEL_ID_SERVER, vnd_op2, NULL, NULL),
};
#endif

#if CONFIG_MAX_NUM_ELEMENT >=4
esp_ble_mesh_model_t vnd_models3[] = {
    ESP_BLE_MESH_VENDOR_MODEL(CID_ESP, ESP_BLE_MESH_VND_MODEL_ID_SERVER, vnd_op3, NULL, NULL),
};
#endif

/*==========================================================================
                        CONFIGURATION ELEMENTS
============================================================================*/
static esp_ble_mesh_elem_t elements[] = {
    ESP_BLE_MESH_ELEMENT(0, root_models, vnd_models),
#if CONFIG_MAX_NUM_ELEMENT >=2
    ESP_BLE_MESH_ELEMENT(0, sig_models1, vnd_models1),
#endif
#if CONFIG_MAX_NUM_ELEMENT >=3
    ESP_BLE_MESH_ELEMENT(0, sig_models2, vnd_models2),
#endif
#if CONFIG_MAX_NUM_ELEMENT >=4
    ESP_BLE_MESH_ELEMENT(0, sig_models3, vnd_models3),
#endif
};

static esp_ble_mesh_comp_t composition = {
    .cid = CID_ESP,
    .element_count = ARRAY_SIZE(elements),
    .elements = elements,
};

static esp_ble_mesh_model_t *sig_model_onoff[CONFIG_MAX_NUM_ELEMENT] = 
{
#if CONFIG_MAX_NUM_ELEMENT >= 1
    [0] = &root_models[1]
#endif
#if CONFIG_MAX_NUM_ELEMENT >= 2
    ,[1] = &sig_models1[0]
#endif
#if CONFIG_MAX_NUM_ELEMENT >= 3
    ,[2] = &sig_models2[0]
#endif
#if CONFIG_MAX_NUM_ELEMENT >= 4
    ,[3] = &sig_models3[0]
#endif
};

/* Disable OOB security for SILabs Android app */
static esp_ble_mesh_prov_t provision = {
    .uuid = dev_uuid,
#if 0
    .output_size = 4,
    .output_actions = ESP_BLE_MESH_DISPLAY_NUMBER,
    .input_size = 4,
    .input_actions = ESP_BLE_MESH_PUSH,
#else
    .output_size = 0,
    .output_actions = 0,
#endif
};

/**
 * @brief hàm log thông báo thiết bị đã provision thành công
 * 
 * @param net_idx index network
 * @param addr  địa chỉ thiết bị
 * @param flags 
 * @param iv_index chỉ số IV
 */
static void prov_complete(uint16_t net_idx, uint16_t addr, uint8_t flags, uint32_t iv_index)
{
    ESP_LOGI(TAG, "net_idx: 0x%04x, addr: 0x%04x", net_idx, addr);
    ESP_LOGI(TAG, "flags: 0x%02x, iv_index: 0x%08" PRIx32, flags, iv_index);
}

/**
 * @brief hàm ví dụ điều khiển trạng thái led
 * 
 * @param ctx chứa địa chỉ của địa chỉ nguồn và địa chỉ đích
 * @param onoff trạng thái điều khiển
 */
static void example_change_led_state(esp_ble_mesh_model_t *model,
                                     esp_ble_mesh_msg_ctx_t *ctx, uint8_t onoff)
{
    uint16_t primary_addr = esp_ble_mesh_get_primary_element_address();
    uint8_t elem_count = esp_ble_mesh_get_element_count();
    uint8_t i;

    if (ESP_BLE_MESH_ADDR_IS_UNICAST(ctx->recv_dst)) {
        for (i = 0; i < elem_count; i++) {
            if (ctx->recv_dst == (primary_addr + i)) {
                //control 1 element
                uint16_t data_send = (i<<8) | onoff; 
                ESP_LOGI(TAG, "control ele: %d, addr: 0x%04X, stt: %d\n", i, ctx->recv_dst, onoff);
                esp_event_post_to(ble_mesh_event_loop, EVENT_MESH_GENERIC_MODEL, EVENT_CONTROLL_ONOFF_BY_UNICAST_ADDR, &data_send, sizeof(data_send), pdMS_TO_TICKS(10));
            }
        }
    } else if (ESP_BLE_MESH_ADDR_IS_GROUP(ctx->recv_dst)) {
        if (esp_ble_mesh_is_model_subscribed_to_group(model, ctx->recv_dst)) {
            // control group
            uint8_t ele_idx = model->element->element_addr - primary_addr;
            ESP_LOGI(TAG, "control group addr: 0x%04X, stt: %d\n", ctx->recv_dst, onoff);
            uint16_t data_send = (ele_idx<<8) | onoff;
            esp_event_post_to(ble_mesh_event_loop, EVENT_MESH_GENERIC_MODEL, EVENT_CONTROLL_ONOFF_BY_GROUP_ADDR, &data_send, sizeof(data_send), pdMS_TO_TICKS(10));
        }
    } else if (ctx->recv_dst == 0xFFFF) {
        // control all element
        ESP_LOGI(TAG, "control all ele, stt: %d\n", onoff);
        esp_event_post_to(ble_mesh_event_loop, EVENT_MESH_GENERIC_MODEL, EVENT_CONTROLL_ONOFF_ALL, &onoff, 1, pdMS_TO_TICKS(10));
    }
}

/**
 * @brief hàm reset cứng thiết bị, hàm này sẽ xóa toàn bộ thông tin mạng mesh và khởi động lại chip
 * 
 */
esp_err_t ble_mesh_kick_out(void)
{
    ESP_LOGW(TAG, "==============>>> RESET MESH NETWORK <<<==============");
    vTaskDelay(500 / portTICK_PERIOD_MS);
    esp_err_t err = esp_ble_mesh_node_local_reset(); // reset 
    vTaskDelay(500 / portTICK_PERIOD_MS);
    // esp_restart();
    return err;
}

/**
 * @brief hàm xử lý điều khiển onoff khi set get_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP
 * 
 * @param model 
 * @param ctx 
 * @param set 
 */
static void example_handle_gen_onoff_msg(esp_ble_mesh_model_t *model,
                                         esp_ble_mesh_msg_ctx_t *ctx,
                                         esp_ble_mesh_server_recv_gen_onoff_set_t *set)
{
    esp_ble_mesh_gen_onoff_srv_t *srv = (esp_ble_mesh_gen_onoff_srv_t *)model->user_data;

    switch (ctx->recv_op) {
    case ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET:
        esp_ble_mesh_server_model_send_msg(model, ctx,
            ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_STATUS, sizeof(srv->state.onoff), &srv->state.onoff);
        break;
    case ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET:
    case ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK:
        if (set->op_en == false) {
            srv->state.onoff = set->onoff;
        } else {
            /* TODO: Delay and state transition */
            srv->state.onoff = set->onoff;
        }
        if (ctx->recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET) {
            esp_ble_mesh_server_model_send_msg(model, ctx,
                ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_STATUS, sizeof(srv->state.onoff), &srv->state.onoff);
        }
        esp_ble_mesh_model_publish(model, ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_STATUS,
            sizeof(srv->state.onoff), &srv->state.onoff, ROLE_NODE);
        example_change_led_state(model, ctx, srv->state.onoff);
        break;
    default:
        break;
    }
}

/**
 * @brief hàm callback xử lý quá trình provision
 * 
 * @param event các sự kiện liên quan tới provision
 * @param param giá trị truyền đến
 */
static void example_ble_mesh_provisioning_cb(esp_ble_mesh_prov_cb_event_t event,
                                             esp_ble_mesh_prov_cb_param_t *param)
{
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
        prov_complete(param->node_prov_complete.net_idx, param->node_prov_complete.addr, //NOTE event provision complete
            param->node_prov_complete.flags, param->node_prov_complete.iv_index);
        esp_event_post_to(ble_mesh_event_loop, EVENT_MESH_PROVISIONING, EVENT_MESH_PROVISION_COMPLETE, NULL, 0, pdMS_TO_TICKS(10));
        break;
    case ESP_BLE_MESH_NODE_PROV_RESET_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_RESET_EVT");
        esp_event_post_to(ble_mesh_event_loop, EVENT_MESH_PROVISIONING, EVENT_MESH_PROVISION_RESET, NULL, 0, pdMS_TO_TICKS(10));
        //NOTE reset => kick out
        // ble_mesh_kick_out();
        break;
    case ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT, err_code %d", param->node_set_unprov_dev_name_comp.err_code);
        break;
    default:
        break;
    }
}

/**
 * @brief hàm callback xử lý các sự kiện liên quan đến Binding, group, ...
 * 
 * @param event các sự kiện liên quan
 * @param param 
 */
static void example_ble_mesh_config_server_cb(esp_ble_mesh_cfg_server_cb_event_t event,
                                              esp_ble_mesh_cfg_server_cb_param_t *param)
{
    if (event == ESP_BLE_MESH_CFG_SERVER_STATE_CHANGE_EVT)
    {
        switch (param->ctx.recv_op)
        {
        case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD");
            ESP_LOGI(TAG, "net_idx 0x%04x, app_idx 0x%04x",
                     param->value.state_change.appkey_add.net_idx,
                     param->value.state_change.appkey_add.app_idx);
            ESP_LOG_BUFFER_HEX("AppKey", param->value.state_change.appkey_add.app_key, 16);
            esp_event_post_to(ble_mesh_event_loop, EVENT_MESH_CONFIG_SERVER, EVENT_MESH_ADD_APP_KEY, NULL, 0, pdMS_TO_TICKS(10));
            break;
        case ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND");
            ESP_LOGI(TAG, "elem_addr 0x%04x, app_idx 0x%04x, cid 0x%04x, mod_id 0x%04x",
                     param->value.state_change.mod_app_bind.element_addr,
                     param->value.state_change.mod_app_bind.app_idx,
                     param->value.state_change.mod_app_bind.company_id,
                     param->value.state_change.mod_app_bind.model_id);
            esp_event_post_to(ble_mesh_event_loop, EVENT_MESH_CONFIG_SERVER, EVENT_MESH_BIND_ALL, NULL, 0, pdMS_TO_TICKS(10));
            break;
        case ESP_BLE_MESH_MODEL_OP_MODEL_SUB_ADD:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_MODEL_SUB_ADD");
            ESP_LOGI(TAG, "elem_addr 0x%04x, sub_addr 0x%04x, cid 0x%04x, mod_id 0x%04x",
                     param->value.state_change.mod_sub_add.element_addr,
                     param->value.state_change.mod_sub_add.sub_addr,
                     param->value.state_change.mod_sub_add.company_id,
                     param->value.state_change.mod_sub_add.model_id);
            uint16_t sub_addr = param->value.state_change.mod_sub_add.sub_addr;
            ble_mesh_add_group(param->value.state_change.mod_sub_add.sub_addr);
            esp_event_post_to(ble_mesh_event_loop, EVENT_MESH_CONFIG_SERVER, EVENT_MESH_SUB_ADD_GROUP, &sub_addr, sizeof(sub_addr), pdMS_TO_TICKS(10));
            break;
        case ESP_BLE_MESH_MODEL_OP_MODEL_SUB_DELETE:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_MODEL_SUB_DELETE delete group id: %04x", param->value.state_change.mod_sub_delete.sub_addr);
            uint16_t gr_addr = param->value.state_change.mod_sub_delete.sub_addr;
            ble_mesh_del_group(param->value.state_change.mod_sub_delete.sub_addr);
            esp_event_post_to(ble_mesh_event_loop, EVENT_MESH_CONFIG_SERVER, EVENT_MESH_SUB_DELETE_GROUP, &gr_addr, sizeof(gr_addr), pdMS_TO_TICKS(10));
        default:
            break;
        }
    }
}

/*vender*/
/**
 * @brief hàm callback xử lý các sự kiện thuộc vender model
 * 
 * @param event 
 * @param param 
 */
static void example_ble_mesh_custom_model_cb(esp_ble_mesh_model_cb_event_t event,
                                             esp_ble_mesh_model_cb_param_t *param)
{
    // ESP_LOGI(TAG, "src 0x%04x, dst 0x%04x",
    //          param->model_operation.ctx->addr, param->model_operation.ctx->recv_dst);
    switch (event)
    {
    case ESP_BLE_MESH_MODEL_OPERATION_EVT:
        if (param->model_operation.opcode == RD_OPCODE_E0)
        {
            //NOTE todo
            if(handle_mess_opcode_E0) handle_mess_opcode_E0((ble_mesh_cb_param_t)param);
        }else if (param->model_operation.opcode == RD_OPCODE_E2)
        {
            //NOTE todo
            if(handle_mess_opcode_E2) handle_mess_opcode_E2((ble_mesh_cb_param_t)param);
        }

        break;
    case ESP_BLE_MESH_MODEL_SEND_COMP_EVT: // RD_NOTE: log send msg from element
        if (param->model_send_comp.err_code)
        {
            ESP_LOGE(TAG, "Failed to send message 0x%06" PRIx32, param->model_send_comp.opcode);
            break;
        }
        ESP_LOGI(TAG, "Element: %u, Send rsp_opcode: 0x%06" PRIx32, param->model_send_comp.model->element_idx, param->model_send_comp.opcode); // RD_NOTE device rsp
        break;
    default:
        break;
    }
}

/**
 * @brief hàm callback xử lý các sự kiện thuộc Generic model
 * 
 * @param event 
 * @param param 
 */
static void example_ble_mesh_generic_server_cb(esp_ble_mesh_generic_server_cb_event_t event,
                                               esp_ble_mesh_generic_server_cb_param_t *param)
{
    esp_ble_mesh_gen_onoff_srv_t *srv;
    ESP_LOGI(TAG, "event 0x%02x, opcode 0x%04" PRIx32 ", src 0x%04x, dst 0x%04x",
        event, param->ctx.recv_op, param->ctx.addr, param->ctx.recv_dst);

    switch (event) {
    case ESP_BLE_MESH_GENERIC_SERVER_STATE_CHANGE_EVT:{ //NOTE set_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP
        ESP_LOGI(TAG, "ESP_BLE_MESH_GENERIC_SERVER_STATE_CHANGE_EVT");
        if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET ||
            param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK)
        {
            ESP_LOGI(TAG, "onoff 0x%02x", param->value.state_change.onoff_set.onoff);
            //NOTE todo
            example_change_led_state(param->model, &param->ctx, param->value.state_change.onoff_set.onoff);
        }

        break;
    }
    case ESP_BLE_MESH_GENERIC_SERVER_RECV_GET_MSG_EVT: //NOTE get_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP
        ESP_LOGI(TAG, "ESP_BLE_MESH_GENERIC_SERVER_RECV_GET_MSG_EVT");
        if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET) {
            srv = (esp_ble_mesh_gen_onoff_srv_t *)param->model->user_data;
            ESP_LOGI(TAG, "onoff 0x%02x", srv->state.onoff);
            example_handle_gen_onoff_msg(param->model, &param->ctx, NULL);
        }
        break;
    case ESP_BLE_MESH_GENERIC_SERVER_RECV_SET_MSG_EVT: //NOTE set_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP
        ESP_LOGI(TAG, "ESP_BLE_MESH_GENERIC_SERVER_RECV_SET_MSG_EVT");
        if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET ||
            param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK) {
            ESP_LOGI(TAG, "onoff 0x%02x, tid 0x%02x", param->value.set.onoff.onoff, param->value.set.onoff.tid);
            if (param->value.set.onoff.op_en) {
                ESP_LOGI(TAG, "trans_time 0x%02x, delay 0x%02x",
                    param->value.set.onoff.trans_time, param->value.set.onoff.delay);
            }
            example_handle_gen_onoff_msg(param->model, &param->ctx, &param->value.set.onoff);
        }
        break;
    default:
        ESP_LOGE(TAG, "Unknown Generic Server event 0x%02x", event);
        break;
    }
}

#if CONFIG_ENABLE_LIGHT_DIM_CCT
/**
 * @brief hàm callback xử lý các sự kiện Lighting model 
 * 
 * @param event 
 * @param param 
 */
static void example_ble_mesh_lighting_server_cb(esp_ble_mesh_lighting_server_cb_event_t event,
                                                   esp_ble_mesh_lighting_server_cb_param_t *param){
    ESP_LOGI(TAG, "event 0x%02x, opcode 0x%04" PRIx32 ", src 0x%04x, dst 0x%04x",
                event, param->ctx.recv_op, param->ctx.addr, param->ctx.recv_dst);
    switch(event){
        case ESP_BLE_MESH_LIGHTING_SERVER_STATE_CHANGE_EVT:{
            ESP_LOGI(TAG, "ESP_BLE_MESH_LIGHTING_SERVER_STATE_CHANGE_EVT");
            if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_LIGHT_LIGHTNESS_SET ||
                param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_LIGHT_LIGHTNESS_SET_UNACK)
            {
                ESP_LOGI(TAG, "lightness 0x%04x", param->value.state_change.lightness_set.lightness);
                //Todo
                uint16_t lightness = param->value.state_change.lightness_set.lightness;
                esp_event_post_to(ble_mesh_event_loop, EVENT_MESH_LIGHTING_MODEL, EVENT_CONTROLL_LIGHTNESS, &lightness, sizeof(lightness), pdMS_TO_TICKS(10));
            }  
            if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_LIGHT_CTL_TEMPERATURE_SET ||
                param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_LIGHT_CTL_TEMPERATURE_SET_UNACK)
            {
                ESP_LOGI(TAG, "cct 0x%04x", param->value.state_change.ctl_temp_set.temperature);
                //Todo
                uint16_t cct = param->value.state_change.ctl_temp_set.temperature;
                esp_event_post_to(ble_mesh_event_loop, EVENT_MESH_LIGHTING_MODEL, EVENT_CONTROLL_CTL_TEMPERATURE, &cct, sizeof(cct), pdMS_TO_TICKS(10));
            }            
            break;
        }
        default:
            break;
    }
    
}
#endif

/* SCENE MODEL*/
void ble_mesh_time_scene_server_callback(esp_ble_mesh_time_scene_server_cb_event_t event,
                                         esp_ble_mesh_time_scene_server_cb_param_t *param)
{
    switch (event)
    {
    case ESP_BLE_MESH_TIME_SCENE_SERVER_STATE_CHANGE_EVT:
        uint16_t scene_number = param->value.state_change.scene_store.scene_number;
        if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_SCENE_STORE)
        {
            ESP_LOGI(TAG, "store Scene 0x%04X", scene_number);
            //TODO
            esp_event_post_to(ble_mesh_event_loop, EVENT_MESH_SCENE_MODEL, EVENT_MESH_STORE_SCENE, &scene_number, sizeof(scene_number), pdMS_TO_TICKS(10));
        }
        else if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_SCENE_DELETE)
        {
            ESP_LOGI(TAG, "delete Scene 0x%04X", scene_number);
            //TODO
            esp_event_post_to(ble_mesh_event_loop, EVENT_MESH_SCENE_MODEL, EVENT_MESH_DELETE_SCENE, &scene_number, sizeof(scene_number), pdMS_TO_TICKS(10));
        }
        else if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_SCENE_RECALL)
        {
            ESP_LOGI(TAG, "recall Scene 0x%04X", scene_number);
            //TODO
            esp_event_post_to(ble_mesh_event_loop, EVENT_MESH_SCENE_MODEL, EVENT_MESH_RECALL_SCENE, &scene_number, sizeof(scene_number), pdMS_TO_TICKS(10));
        }
        break;
    default:
        ESP_LOGW(TAG, "unknown scene event: %d", event);
        break;
    }
}

/*================= BLE ADV ==================*/
// static void custom_ble_scan_cb(esp_ble_mesh_ble_cb_event_t event,
//                                esp_ble_mesh_ble_cb_param_t *param) {
//   if (event == ESP_BLE_MESH_SCAN_BLE_ADVERTISING_PKT_EVT) {
//     int8_t rssi = param->scan_ble_adv_pkt.rssi;
//     uint8_t *adv_data = param->scan_ble_adv_pkt.data;
//     uint16_t adv_len = param->scan_ble_adv_pkt.length;
//     uint8_t *mac = param->scan_ble_adv_pkt.addr;

//     if(adv_len == 15 && adv_data[1] == 0xff){
// #if LOG_RAW_DATA_BLE_ADV
//         ESP_LOGI(TAG, "MAC %02x:%02x:%02x:%02x:%02x:%02x",
//                     mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);   
//         ESP_LOG_BUFFER_HEX(TAG, adv_data, adv_len);
// #endif
//         if(ble_mesh_send_ble_adv)
//             ble_mesh_send_ble_adv(rssi, mac, adv_data, adv_len);
//     }
//   }
// }

/**
 * @brief hàm khởi tạo BLE mesh
 * 
 * @return ESP_OK khởi tạo thành công
 *         còn lại: khởi tạo thất bại
 */
static esp_err_t ble_mesh_init(void)
{
    esp_err_t err = ESP_OK;

    esp_ble_mesh_register_prov_callback(example_ble_mesh_provisioning_cb);
    esp_ble_mesh_register_config_server_callback(example_ble_mesh_config_server_cb);
    esp_ble_mesh_register_custom_model_callback(example_ble_mesh_custom_model_cb);     // vender
    esp_ble_mesh_register_generic_server_callback(example_ble_mesh_generic_server_cb);  // sigmesh: generic model
    // esp_ble_mesh_register_lighting_server_callback(example_ble_mesh_lighting_server_cb);// sigmesh: lighting model
    esp_ble_mesh_register_time_scene_server_callback(ble_mesh_time_scene_server_callback); // scene model
    // esp_ble_mesh_register_ble_callback(custom_ble_scan_cb);

    err = esp_ble_mesh_init(&provision, &composition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize mesh stack (err %d)", err);
        return err;
    }

    err = esp_ble_mesh_node_prov_enable((esp_ble_mesh_prov_bearer_t)(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable mesh node (err %d)", err);
        return err;
    }

    ESP_LOGI(TAG, "BLE Mesh Node initialized");
    return err;
}


#ifdef CONFIG_BT_NIMBLE_ENABLED
static SemaphoreHandle_t mesh_sem;
static uint8_t own_addr_type;
void ble_store_config_init(void);
static uint8_t addr_val[6] = {0};

void ble_mesh_get_dev_uuid(uint8_t *dev_uuid)
{
    if (dev_uuid == NULL) {
        ESP_LOGE(TAG, "%s, Invalid device uuid", __func__);
        return;
    }

    /* Copy device address to the device uuid with offset equals to 2 here.
     * The first two bytes is used for matching device uuid by Provisioner.
     * And using device address here is to avoid using the same device uuid
     * by different unprovisioned devices.
     */
    memcpy(dev_uuid + 2, addr_val, BD_ADDR_LEN);
}

static void mesh_on_reset(int reason)
{
    ESP_LOGI(TAG, "Resetting state; reason=%d", reason);
}

static void mesh_on_sync(void)
{
    int rc;

    rc = ble_hs_util_ensure_addr(0);
    assert(rc == 0);

    /* Figure out address to use while advertising (no privacy for now) */
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGI(TAG, "error determining address type; rc=%d", rc);
        return;
    }

    rc = ble_hs_id_copy_addr(own_addr_type, addr_val, NULL);

    xSemaphoreGive(mesh_sem);
}

void mesh_host_task(void *param)
{
    ESP_LOGI(TAG, "BLE Host Task Started");
    /* This function will return only when nimble_port_stop() is executed */
    nimble_port_run();

    nimble_port_freertos_deinit();
}

esp_err_t bluetooth_init(void)
{
    esp_err_t ret;

    mesh_sem = xSemaphoreCreateBinary();
    if (mesh_sem == NULL) {
        ESP_LOGE(TAG, "Failed to create mesh semaphore");
        return ESP_FAIL;
    }

    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init nimble %d ", ret);
        return ret;
    }

    /* Initialize the NimBLE host configuration. */
    ble_hs_cfg.reset_cb = mesh_on_reset;
    ble_hs_cfg.sync_cb = mesh_on_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    /* XXX Need to have template for store */
    ble_store_config_init();

    nimble_port_freertos_init(mesh_host_task);

    xSemaphoreTake(mesh_sem, portMAX_DELAY);

    return ESP_OK;
}
#endif /* CONFIG_BT_NIMBLE_ENABLED */

#ifdef CONFIG_BT_BLUEDROID_ENABLED
void ble_mesh_get_dev_uuid(uint8_t *dev_uuid)
{
    if (dev_uuid == NULL) {
        ESP_LOGE(TAG, "%s, Invalid device uuid", __func__);
        return;
    }
    memcpy(dev_uuid + 2, esp_bt_dev_get_address(), BD_ADDR_LEN);
}

esp_err_t bluetooth_init(void)
{
    esp_err_t ret;

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "%s initialize controller failed", __func__);
        return ret;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "%s enable controller failed", __func__);
        return ret;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "%s init bluetooth failed", __func__);
        return ret;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "%s enable bluetooth failed", __func__);
        return ret;
    }

    return ret;
}
#endif

static bool is_ble_mesh_init = false;
/**
 * @brief hàm xử lý chính của chương trình
 * 
 */
void rd_ble_mesh_init(void)
{
    esp_err_t err;

    ESP_LOGI(TAG, "Initializing...");

#if !CONFIG_USE_SDK_ROGO
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    err = bluetooth_init();
    if (err) {
        ESP_LOGE(TAG, "esp32_bluetooth_init failed (err %d)", err);
        return;
    }
#endif

    ble_mesh_get_dev_uuid(dev_uuid);
    init_scene_values(); //NOTE init scene value
    sensor_model_init(); // init sensor value
    /* Initialize the Bluetooth Mesh Subsystem */
    err = ble_mesh_init();
    if (err) {
        ESP_LOGE(TAG, "Bluetooth mesh init failed (err %d)", err);
    }
    if (esp_ble_mesh_node_is_provisioned())
    {
        ESP_LOGW(TAG, "provisioned");
    }
    else
    {
        ESP_LOGW(TAG, "un provision");
    }
    // esp_ble_mesh_ble_scan_param_t scan_param = {0 };
    // err = esp_ble_mesh_start_ble_scanning(&scan_param);
    // if (err != ESP_OK) {
    //     ESP_LOGE(TAG, "Start scanning failed: %d", err);
    // }
    is_ble_mesh_init= true;
}

void rd_continue_ble_mesh(void)
{
    if (!is_ble_mesh_init)
    {
        is_ble_mesh_init = true;
        ESP_LOGW(TAG, "continue BLE mesh");

        esp_err_t err = esp_ble_mesh_init(&provision, &composition);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to initialize mesh stack");
        }

        if (esp_ble_mesh_node_is_provisioned())
        {
            printf("device provisioned\n");
        }
        else
        {
            printf("device un provisioned, esp_ble_mesh_node_prov_enable\n");
            err = esp_ble_mesh_node_prov_enable((esp_ble_mesh_prov_bearer_t)(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT));
            vTaskDelay(pdMS_TO_TICKS(100));
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to enable mesh node");
            }
        }
        // esp_ble_mesh_ble_scan_param_t scan_param = {0};
        // err = esp_ble_mesh_start_ble_scanning(&scan_param);
        // if (err != ESP_OK) {
        //     ESP_LOGE(TAG, "Start scanning failed: %d", err);
        // }
    }
}

void rd_suspend_ble_mesh(void)
{
    if (is_ble_mesh_init)
    {
        is_ble_mesh_init = false;
        esp_err_t err = ESP_OK;//esp_ble_mesh_stop_ble_scanning();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Stop scanning failed: %d", err);
        }
        esp_ble_mesh_node_prov_disable((esp_ble_mesh_prov_bearer_t)(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT));
        esp_ble_mesh_deinit_param_t param = {
            .erase_flash = false // hoặc true nếu muốn reset
        };
        esp_ble_mesh_deinit(&param);

        ESP_LOGW(TAG, "suspend BLE mesh");

    }
}

/*=====================================================================================
                                RANG DONG IMPLEMENT
=====================================================================================*/
static void dummy_ble_mesh_handler(void *arg,
                             esp_event_base_t base,
                             int32_t id,
                             void *data)
{
    // do nothing
}

esp_err_t rd_ble_mesh_event_init(void){
    esp_event_loop_args_t loop_args = {
        .queue_size = 10,
        .task_name = "rd_ble_mesh_event_loop",
        .task_priority = uxTaskPriorityGet(NULL),
        .task_stack_size = 2048*2,
        .task_core_id = tskNO_AFFINITY
    };
    return esp_event_loop_create(&loop_args, &ble_mesh_event_loop);
}

esp_err_t rd_ble_mesh_register_event_handler(esp_event_base_t evt_base, esp_event_handler_t event_handler){
    if(!event_handler){
        event_handler = dummy_ble_mesh_handler;
    }
    return esp_event_handler_register_with(ble_mesh_event_loop, evt_base, ESP_EVENT_ANY_ID, event_handler, NULL);
}


void rd_ble_mesh_register_cb_handle_mess_opcode_E0(rd_handle_message_opcode_vender cb){
    if(cb) handle_mess_opcode_E0 = cb;
}

void rd_ble_mesh_register_cb_handle_mess_opcode_E2(rd_handle_message_opcode_vender cb){
    if(cb) handle_mess_opcode_E2 = cb;
}

void rd_ble_mesh_register_cb_send_ble_adv(rd_ble_mesh_send_ble_adv cb){
    if(cb) ble_mesh_send_ble_adv = cb;
}

void ble_mesh_get_mess_buf(ble_mesh_cb_param_t param, uint8_t **buff, uint16_t *len){
    esp_ble_mesh_model_cb_param_t *cb_par = (esp_ble_mesh_model_cb_param_t *)param;
    *buff = cb_par->model_operation.msg;
    *len = cb_par->model_operation.length;
}

uint32_t ble_mesh_get_opcode(ble_mesh_cb_param_t param){
    esp_ble_mesh_model_cb_param_t *cb_par = (esp_ble_mesh_model_cb_param_t *)param;
    return cb_par->model_operation.opcode;
}

// typedef enum {
//     BTC_BLE_MESH_ACT_MODEL_PUBLISH,
//     BTC_BLE_MESH_ACT_SERVER_MODEL_SEND,
//     BTC_BLE_MESH_ACT_CLIENT_MODEL_SEND,
//     BTC_BLE_MESH_ACT_SERVER_MODEL_UPDATE_STATE,
// } btc_ble_mesh_model_act_t;

/*=============================================================================
                        BLE MESH MODEL SEND MESSAGE
=============================================================================*/
static esp_err_t rd_ble_mesh_model_publish(esp_ble_mesh_model_t *model, uint32_t opcode,
                                     uint16_t length, uint8_t *data)
{
    return esp_ble_mesh_model_publish(model, opcode, length, data, ROLE_NODE);
}

static esp_err_t rd_ble_mesh_server_model_send_msg(esp_ble_mesh_model_t *model,
                                             esp_ble_mesh_msg_ctx_t *ctx,
                                             uint32_t opcode,
                                             uint16_t length, uint8_t *data)
{
    return esp_ble_mesh_server_model_send_msg(model, ctx, opcode, length, data);
}

static esp_err_t rd_ble_mesh_client_model_send_msg(esp_ble_mesh_model_t *model,
                                             esp_ble_mesh_msg_ctx_t *ctx,
                                             uint32_t opcode,
                                             uint16_t length, uint8_t *data,
                                             int32_t msg_timeout, bool need_rsp)
{
    return esp_ble_mesh_client_model_send_msg(model, ctx, opcode, length, data, msg_timeout, need_rsp, ROLE_NODE);
}

static esp_err_t rd_ble_mesh_server_model_update_state(esp_ble_mesh_model_t *model,
                                                 esp_ble_mesh_server_state_type_t type,
                                                 esp_ble_mesh_server_state_value_t *value)
{
    return esp_ble_mesh_server_model_update_state(model, type, value);
}
/*------------------------------------------------------------------------------------------------*/

esp_err_t ble_mesh_client_model_scene_send_msg(uint32_t opcode, uint16_t dst_addr, uint8_t *par, uint16_t len){
    esp_ble_mesh_msg_ctx_t ctx;
    ctx.net_idx = 0x0000;
    ctx.app_idx = 0x0000;
    ctx.addr = dst_addr; 
    ctx.send_ttl = ESP_BLE_MESH_TTL_DEFAULT;
    ctx.send_rel = 0;  
    return rd_ble_mesh_client_model_send_msg(scene_client.model, &ctx, opcode, len, par, 0, false); // ~ &root_model[7]
}

esp_err_t ble_mesh_model_sensor_send_msg(uint8_t *par, uint16_t len){
    sensor_srv.model->pub->publish_addr = GW_ADDR;
    return rd_ble_mesh_model_publish(sensor_srv.model, BLE_MESH_MODEL_OP_SENSOR_STATUS, len, par);
}

esp_err_t ble_mesh_send_opcode_vender_E1(uint8_t *par, uint16_t len){
    esp_ble_mesh_msg_ctx_t ctx;
    ctx.net_idx = 0x0000;
    ctx.app_idx = 0x0000;
    ctx.addr = GW_ADDR; 
    ctx.send_ttl = ESP_BLE_MESH_TTL_DEFAULT;
    ctx.send_rel = 0;  
    return rd_ble_mesh_server_model_send_msg(&vnd_models[0], &ctx, RD_OPCODE_RSP_E0, len, par); 
}

esp_err_t ble_mesh_send_opcode_vender_E3(uint8_t *par, uint16_t len){
    esp_ble_mesh_msg_ctx_t ctx;
    ctx.net_idx = 0x0000;
    ctx.app_idx = 0x0000;
    ctx.addr = GW_ADDR; 
    ctx.send_ttl = ESP_BLE_MESH_TTL_DEFAULT;
    ctx.send_rel = 0;  
    return rd_ble_mesh_server_model_send_msg(&vnd_models[0], &ctx, RD_OPCODE_RSP_E2, len, par);
}

esp_err_t ble_mesh_update_state_onoff(uint8_t eleIdx, uint8_t onoff){
    esp_ble_mesh_server_state_value_t data = {0};
    data.gen_onoff.onoff = onoff;
    if((*sig_model_onoff[eleIdx]).pub->publish_addr != GW_ADDR){
        (*sig_model_onoff[eleIdx]).pub->publish_addr = GW_ADDR;
        // ~ root_models[1].pub->publish_addr = 0x0001;
        // ~ onoff_server_0.model->pub->publish_addr = 0x0001;
    }
    return rd_ble_mesh_server_model_update_state(sig_model_onoff[eleIdx], ESP_BLE_MESH_GENERIC_ONOFF_STATE, &data);
}

esp_err_t ble_mesh_rsp_opcode_vender_E0(ble_mesh_cb_param_t param, uint8_t *par, uint8_t len){
    esp_ble_mesh_model_cb_param_t *cb_par = (esp_ble_mesh_model_cb_param_t *)param;
    esp_err_t err = rd_ble_mesh_server_model_send_msg(vnd_models,
                                                        cb_par->model_operation.ctx, RD_OPCODE_RSP_E0,
                                                        len, par);
    if (err)
    { 
        ESP_LOGE(TAG, "Failed to send message 0x%06x", RD_OPCODE_RSP_E0);
    }
    return err;   
}
esp_err_t ble_mesh_rsp_opcode_vender_E2(ble_mesh_cb_param_t param, uint8_t *par, uint8_t len){
    esp_ble_mesh_model_cb_param_t *cb_par = (esp_ble_mesh_model_cb_param_t *)param;
    esp_err_t err = rd_ble_mesh_server_model_send_msg(vnd_models,
                                                        cb_par->model_operation.ctx, RD_OPCODE_RSP_E2,
                                                        len, par);
    if (err)
    { 
        ESP_LOGE(TAG, "Failed to send message 0x%06x", RD_OPCODE_RSP_E2);
    }
    return err; 
}

uint8_t ble_mesh_get_element_index(ble_mesh_cb_param_t param){
    esp_ble_mesh_model_cb_param_t *cb_par = (esp_ble_mesh_model_cb_param_t *)param;
    uint16_t primary_addr = esp_ble_mesh_get_primary_element_address();
    uint16_t addr_dst = cb_par->model_operation.ctx->recv_dst;
    uint8_t ele_idx = addr_dst - primary_addr;
    return ele_idx;
}

void ble_mesh_add_group(uint16_t id_group){
    uint16_t primary_addr = esp_ble_mesh_get_primary_element_address();
    uint16_t model_id = ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_SRV;
    uint16_t company_id = 0xffff; // SIG_MODEL: company_id = 0xffff

    ESP_LOGI(TAG, "add group: 0x%04x\n", id_group);
    esp_ble_mesh_model_subscribe_group_addr(primary_addr, company_id, model_id, id_group);
}

void ble_mesh_del_group(uint16_t id_group){
    uint16_t primary_addr = esp_ble_mesh_get_primary_element_address();
    uint16_t model_id = ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_SRV;
    uint16_t company_id = 0xffff; // SIG_MODEL: company_id = 0xffff

    ESP_LOGI(TAG, "delete group: 0x%04x\n", id_group);
    esp_ble_mesh_model_unsubscribe_group_addr(primary_addr, company_id, model_id, id_group);

}

void ble_mesh_get_mac(uint8_t mac[6])
{
    if (mac == NULL) {
        ESP_LOGE(TAG, "%s, Invalid mac address", __func__);
        return;
    }
    // memcpy(mac, esp_bt_dev_get_address(), 6);
}

uint16_t ble_mesh_get_primary_addr(void){
    return esp_ble_mesh_get_primary_element_address();
}

uint16_t ble_mesh_get_src_addr(ble_mesh_cb_param_t param){
    esp_ble_mesh_model_cb_param_t *cb_par = (esp_ble_mesh_model_cb_param_t *)param;
    return cb_par->model_operation.ctx->addr;
}

uint16_t ble_mesh_get_dst_addr(ble_mesh_cb_param_t param){
    esp_ble_mesh_model_cb_param_t *cb_par = (esp_ble_mesh_model_cb_param_t *)param;
    return cb_par->model_operation.ctx->recv_dst;
}

void ble_mesh_set_gw_addr(uint16_t addr){
    GW_ADDR = addr;
}

uint16_t ble_mesh_get_gw_addr(void){
    return GW_ADDR;
}

bool ble_mesh_is_provisioned(void){
    return esp_ble_mesh_node_is_provisioned();
}



