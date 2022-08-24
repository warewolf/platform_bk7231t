/**
 * @file tuya_os_adapt_bt.h
 * @brief BLE操作接口
 *
 * @copyright Copyright(C),2018-2020, 涂鸦科技 www.tuya.com
 *
 */
#include "tuya_os_adapter.h"
#include "tuya_os_adapter_error_code.h"
#include "tuya_os_adapt_bt.h"
#include "tuya_os_adapt_wifi.h"
#include "tuya_os_adapt_output.h"
#include "tuya_os_adapt_memory.h"
#include "tuya_os_adapt_system.h"
#include "tuya_os_adapt_thread.h"
#include "tuya_os_adapt_semaphore.h"


#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "rwip_config.h"
#include "ble_api.h"
#include "ble_pub.h"
#include "app_task.h"
#include "mcu_ps_pub.h"
#include "application.h"


#if defined(TY_BT_MOD) && TY_BT_MOD == 1

unsigned short ntf_enable;

#define TY_BT_VER10_PROTOCOL_TYPE           0xff
#define TY_BT_VER11_PROTOCOL_TYPE_IDENTIFY  0x01
#define TY_BT_VER11_PROTOCOL_TYPE_SER_UUID  0x02
#define TY_BT_VER11_PROTOCOL_TYPE_SER_DATA  0x16

#define BK_ATT_DECL_PRIMARY_SERVICE_128     {0x00,0x28,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
#define BK_ATT_DECL_CHARACTERISTIC_128      {0x03,0x28,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
#define BK_ATT_DESC_CLIENT_CHAR_CFG_128     {0x02,0x29,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
#define SCAN_SLEEP_MS 10


#if defined(TUYA_BLE_VER) && (TUYA_BLE_VER == 4)

//Write Characteristic UUID：00000001-0000-1001-8001-00805F9B07D0
#define WRITE_REQ_CHARACTERISTIC_128_v4        {0xD0,0x07,0x9B,0x5F,0x80,0x00,  0x01,0x80,  0x01,0x10,  0,0,  0x01,0,0,0}
//Notify Characteristic UUID：00000002-0000-1001-8001-00805F9B07D0
#define NOTIFY_CHARACTERISTIC_128_v4           {0xD0,0x07,0x9B,0x5F,0x80,0x00,  0x01,0x80,  0x01,0x10,  0,0,  0x02,0,0,0}

static const uint8_t tuya_svc_uuid_v4[16] = {0x50, 0xFD, 0, 0, 0x34, 0x56, 0, 0, 0, 0, 0x28, 0x37, 0, 0, 0, 0};

bk_attm_desc_t btl_att_db[6] = {

    //  Service Declaration
    [0]        =   {BK_ATT_DECL_PRIMARY_SERVICE_128,  BK_PERM_SET(RD, ENABLE), 0, 0},

    //  Level Characteristic Declaration
    [1]        =   {BK_ATT_DECL_CHARACTERISTIC_128,   BK_PERM_SET(RD, ENABLE), 0, 0},

    //  Level Characteristic Value
    [2]        =   {WRITE_REQ_CHARACTERISTIC_128_v4,   BK_PERM_SET(WRITE_REQ, ENABLE) | BK_PERM_SET(WRITE_COMMAND, ENABLE), BK_PERM_SET(RI, ENABLE) | BK_PERM_SET(UUID_LEN, UUID_128), 256},
    [3]        =   {BK_ATT_DECL_CHARACTERISTIC_128,   BK_PERM_SET(RD, ENABLE), 0, 0},

    //  Level Characteristic Value
    [4]        =   {NOTIFY_CHARACTERISTIC_128_v4,   BK_PERM_SET(NTF, ENABLE), BK_PERM_SET(RI, ENABLE) | BK_PERM_SET(UUID_LEN, UUID_128), 256},

    //  Level Characteristic - Client Characteristic Configuration Descriptor
    [5]        =   {BK_ATT_DESC_CLIENT_CHAR_CFG_128,  BK_PERM_SET(RD, ENABLE) | BK_PERM_SET(WRITE_REQ, ENABLE), 0, 0},
};
#else

#define WRITE_REQ_CHARACTERISTIC_128        {0x11,0x2b,0,0,0x34,0x56,0,0,0,0,0x28,0x37,0,0,0,0}
#define NOTIFY_CHARACTERISTIC_128           {0x10,0x2b,0,0,0x34,0x56,0,0,0,0,0x28,0x37,0,0,0,0}


static const uint8_t tuya_svc_uuid[16] = {0x10, 0x19, 0, 0, 0x34, 0x56, 0, 0, 0, 0, 0x28, 0x37, 0, 0, 0, 0};

bk_attm_desc_t btl_att_db[6] = {

    //  Service Declaration
    [0]        =   {BK_ATT_DECL_PRIMARY_SERVICE_128,  BK_PERM_SET(RD, ENABLE), 0, 0},

    //  Level Characteristic Declaration
    [1]        =   {BK_ATT_DECL_CHARACTERISTIC_128,   BK_PERM_SET(RD, ENABLE), 0, 0},

    //  Level Characteristic Value
    [2]        =   {WRITE_REQ_CHARACTERISTIC_128,   BK_PERM_SET(WRITE_REQ, ENABLE) | BK_PERM_SET(WRITE_COMMAND, ENABLE), BK_PERM_SET(RI, ENABLE) | BK_PERM_SET(UUID_LEN, UUID_16), 20},
    [3]        =   {BK_ATT_DECL_CHARACTERISTIC_128,   BK_PERM_SET(RD, ENABLE), 0, 0},

    //  Level Characteristic Value
    [4]        =   {NOTIFY_CHARACTERISTIC_128,   BK_PERM_SET(NTF, ENABLE), BK_PERM_SET(RI, ENABLE) | BK_PERM_SET(UUID_LEN, UUID_16), 20},

    //  Level Characteristic - Client Characteristic Configuration Descriptor
    [5]        =   {BK_ATT_DESC_CLIENT_CHAR_CFG_128,  BK_PERM_SET(RD, ENABLE) | BK_PERM_SET(WRITE_REQ, ENABLE), 0, 0},
};
#endif

TY_BT_MSG_CB ty_bt_msg_cb;

unsigned char ble_att_flag = 0;
unsigned char ble_stack_init_ok = 0;
unsigned char ble_scan_flag = 0;
bool if_start_adv_after_disconnect = true;

static ty_bt_scan_info_t *ble_scan_info = NULL;
static bool ble_scan_found = FALSE;

static SEM_HANDLE ble_err_sem = NULL;
THREAD_HANDLE ble_restart_thrd;

static const TUYA_OS_BT_INTF m_tuya_os_bt_intfs = {
    .port_init      = tuya_os_adapt_bt_port_init,
    .port_deinit    = tuya_os_adapt_bt_port_deinit,
    .gap_disconnect = tuya_os_adapt_bt_gap_disconnect,
    .send           = tuya_os_adapt_bt_send,
    .reset_adv      = tuya_os_adapt_bt_reset_adv,
    .get_rssi       = tuya_os_adapt_bt_get_rssi,
    .start_adv      = tuya_os_adapt_bt_start_adv,
    .stop_adv       = tuya_os_adapt_bt_stop_adv,
    .assign_scan    = tuya_os_adapt_bt_assign_scan,
    .scan_init      = tuya_os_adapt_bt_scan_init,
    .start_scan     = tuya_os_adapt_bt_start_scan,
    .stop_scan      = tuya_os_adapt_bt_stop_scan,
};

static void ty_ble_restart_thread(void *parameter);


/**
 * @brief tuya_os_adapt_bt 蓝牙写cb
 * @return none
 */
static void ble_write_callback(write_req_t *write_req)
{
    tuya_ble_data_buf_t data;

    if (write_req->att_idx == 5) {
        ntf_enable = (write_req->value[0]) | (write_req->value[1] << 8);
    } else {
        data.data = write_req->value;
        data.len = write_req->len;

        if (ty_bt_msg_cb != NULL) {
            ty_bt_msg_cb(0, TY_BT_EVENT_RX_DATA, &data);
        }
    }
}

/**
 * @brief tuya_os_adapt_bt 蓝牙读取cb
 * @return none
 */
static unsigned char ble_read_callback(read_req_t *read_req)
{
    if (read_req->att_idx == 5) {
        memcpy(read_req->value, &ntf_enable, sizeof(ntf_enable));
    }
    return 2;
}

/**
 * @brief tuya_os_adapt_bt 蓝牙事件cb
 * @return none
 */
static void ble_event_callback(ble_event_t event, void *param)
{
    switch (event) {
        case BLE_STACK_OK: {
            LOG_NOTICE("STACK INIT OK\r\n");
            struct bk_ble_db_cfg ble_db_cfg;
            ble_db_cfg.att_db = btl_att_db;
            ble_db_cfg.att_db_nb = 6;
            ble_db_cfg.prf_task_id = 0;
            ble_db_cfg.start_hdl = 0;

            ble_db_cfg.svc_perm = BK_PERM_SET(SVC_UUID_LEN, UUID_16);
#if defined(TUYA_BLE_VER) && (TUYA_BLE_VER == 4)
            memcpy(&(ble_db_cfg.uuid[0]), &tuya_svc_uuid_v4[0], 16);
#else
            memcpy(&(ble_db_cfg.uuid[0]), &tuya_svc_uuid[0], 16);
#endif
            bk_ble_create_db(&ble_db_cfg);
        }
        break;
        case BLE_STACK_FAIL:
            LOG_NOTICE("STACK INIT FAIL\r\n");
            break;

        case BLE_CONNECT:
            LOG_NOTICE("BLE CONNECT\r\n");

            if (((struct gapc_connection_req_ind *)param)->sup_to < 200) {
                struct gapc_conn_param conn_param;
                conn_param.intv_max = ((struct gapc_connection_req_ind *)param)->con_interval;
                conn_param.intv_min = ((struct gapc_connection_req_ind *)param)->con_interval;
                conn_param.latency = ((struct gapc_connection_req_ind *)param)->con_latency;
                conn_param.time_out = 600;
                appm_update_param(&conn_param);
            }

            mcu_prevent_set(MCU_PS_BLE_FROBID);
            if (ty_bt_msg_cb != NULL) {
                ty_bt_msg_cb(0, TY_BT_EVENT_CONNECTED, NULL);
            }
            break;

        case BLE_DISCONNECT: {
            LOG_NOTICE("BLE DISCONNECT\r\n");
            ble_att_flag = 0;
            mcu_prevent_clear(MCU_PS_BLE_FROBID);
            if (ty_bt_msg_cb != NULL) {
                ty_bt_msg_cb(0, TY_BT_EVENT_DISCONNECTED, NULL);
            }

            //该模式适用于BK7231T下需要手工disconnect bt
            if (if_start_adv_after_disconnect) {
                tuya_os_adapt_bt_start_adv();
            } else {        //下次自动开启广播
                if_start_adv_after_disconnect = TRUE;
            }
        }
        break;

        case BLE_MTU_CHANGE:
            LOG_NOTICE("BLE_MTU_CHANGE:%d\r\n", *(unsigned short *)param);
            break;
        case BLE_CFG_NOTIFY:
            LOG_NOTICE("BLE_CFG_NOTIFY:%d\r\n", *(unsigned short *)param);
            break;
        case BLE_CFG_INDICATE:
            LOG_NOTICE("BLE_CFG_INDICATE:%d\r\n", *(unsigned short *)param);
            break;
        case BLE_TX_DONE:
            os_printf("BLE_TX_DONE\r\n");
            ble_att_flag = 0;
            break;
        case BLE_GEN_DH_KEY: {
            LOG_NOTICE("BLE_GEN_DH_KEY\r\n");
            LOG_NOTICE("key_len:%d\r\n", ((struct ble_gen_dh_key_ind *)param)->len);
            for (int i = 0; i < ((struct ble_gen_dh_key_ind *)param)->len; i++) {
                LOG_DEBUG("%02x ", ((struct ble_gen_dh_key_ind *)param)->result[i]);
            }
            LOG_NOTICE("\r\n");
        }
        break;
        case BLE_GET_KEY: {
            LOG_NOTICE("BLE_GET_KEY\r\n");
            LOG_NOTICE("pri_len:%d\r\n", ((struct ble_get_key_ind *)param)->pri_len);
            for (int i = 0; i < ((struct ble_get_key_ind *)param)->pri_len; i++) {
                LOG_DEBUG("%02x ", ((struct ble_get_key_ind *)param)->pri_key[i]);
            }
            LOG_DEBUG("\r\n");
        }
        break;
        case BLE_CREATE_DB_OK: {
            LOG_NOTICE("CREATE DB SUCCESS\r\n");

            ble_stack_init_ok = 1;
            if (ty_bt_msg_cb != NULL) {
                ty_bt_msg_cb(0, TY_BT_EVENT_ADV_READY, NULL);
            }

            LOG_NOTICE("!!!!!!!!!!tuya_before_netcfg_cb\r\n");
            appm_start_advertising();
        }
        break;
        case BLE_HW_ERROR: {
            LOG_NOTICE("BLE_HW_ERROR\r\n");
            if (ty_bt_msg_cb != NULL) {
                ty_bt_msg_cb(0, TY_BT_EVENT_DISCONNECTED, NULL);
            }

            int ret = 0;

            static unsigned char init_flag = 0;
            if (!init_flag) {
                ret = tuya_os_adapt_semaphore_create_init(&ble_err_sem, 0, 1);
                ret |= tuya_os_adapt_thread_create(&ble_restart_thrd, "ble_restart", 1024, 5, ty_ble_restart_thread, NULL);
                if (ret !=  OPRT_OS_ADAPTER_OK) {
                    LOG_ERR("ble restart proc err\r\n");
                    return ;
                }
                init_flag = 1;
            }
            tuya_os_adapt_semaphore_post(ble_err_sem);

            init_flag = 0;
        }
        break;
        default:
            LOG_DEBUG("UNKNOW EVENT:%d\r\n", event);
            break;
    }
}

/**
 * @brief  解析ble 广播包数据，获取广播名
 * @return OPERATE_RET
 */
static OPERATE_RET tuya_bt_get_scan_local_name(recv_adv_t *recv_adv, CHAR_T *name)
{
    unsigned char buffer[32];
    unsigned char pos = 0;

    while (pos < recv_adv->data_len) {
        /* Length of the AD structure. */
        unsigned char length = recv_adv->data[pos++];
        unsigned char type;

        if ((length > 0x01) && ((pos + length) <= 31)) {

            /* Copy the AD Data to buffer. */
            memcpy(buffer, recv_adv->data + pos + 1, length - 1);
            /* AD Type, one octet. */
            type = recv_adv->data[pos];

            if (GAP_AD_TYPE_COMPLETE_NAME == type) {
                buffer[length - 1] = '\0';
                strcpy(name, (char *)buffer);
                //PR_DEBUG("GAP_ADTYPE_LOCAL_NAME_XXX: %s\n\r", buffer);
                return OPRT_OS_ADAPTER_OK;
            }
        }
    }

    return OPRT_OS_ADAPTER_COM_ERROR;
}

/**
 * @brief tuya_os_adapt_bt 蓝牙广播包接收数据解析,用于产测
 * @return OPERATE_RET
 */
static void tuya_bt_scan_pre_decode(recv_adv_t *recv_adv)
{
    unsigned char buffer[32];
    unsigned char pos = 0;
    unsigned char mac[6];
    static unsigned char old_data[24];
    unsigned short uuid;

    while (pos < recv_adv->data_len) {
        /* Length of the AD structure. */
        unsigned char length = recv_adv->data[pos++];
        unsigned char type;

        if ((length > 0x01) && ((pos + length) <= 31)) {
            /* Copy the AD Data to buffer. */
            memcpy(buffer, recv_adv->data + pos + 1, length - 1);

            /* AD Type, one octet. */
            type = recv_adv->data[pos];

            switch (type) {
                case TY_BT_VER10_PROTOCOL_TYPE:
                    //老版遥控器定长25字节
                    if (length != 25) {
                        return;
                    }
                    //重复过滤
                    if (0 == memcmp(old_data, buffer, 24)) {
                        return;
                    }
                    memcpy(old_data, buffer, 24);

                    if (ble_scan_info->scan_adv_cb) {
                        /* copy mac */
                        memcpy(mac, recv_adv->adv_addr, 6);
                        //LOG_DEBUG("mac:%x:%x:%x:%x:%x:%x", mac[0],mac[1], mac[2],mac[3], mac[4],mac[5]);
                        ble_scan_info->scan_adv_cb((char *)buffer, length - 1, mac, type);
                    }
                    return;

                case TY_BT_VER11_PROTOCOL_TYPE_IDENTIFY:
                #if 0
                    if (length != 0x02 || buffer[0] != 0x06) {
                        //LOG_ERR("data err, should be 0x06\r\n");
                    }
                #endif
                    break;


                case TY_BT_VER11_PROTOCOL_TYPE_SER_UUID:
                #if 1
                    uuid = ((*(unsigned char *)(buffer)) << 8) + (*(unsigned char *)(buffer + 1));
                    if (length != 0x03 || uuid != 0x50FD) {
                        //LOG_ERR("\r\nuuid err:0x%x,len:%d.", uuid, length);
                        return;
                    }
                #endif
                    break;

                case TY_BT_VER11_PROTOCOL_TYPE_SER_DATA: {
                    if ((buffer[0] != 0x50) && (buffer[1] != 0xFD)) {
                        return;
                    }

                    //协议：len+type+uuid+data，新协议的buff要偏移掉uuid内容(2个字节)
                    if (ble_scan_info->scan_adv_cb) {
                        /* copy mac */
                        memcpy(mac, recv_adv->adv_addr, 6);
                        ble_scan_info->scan_adv_cb((char *)(buffer + 2), length - 3, mac, type);
                    }
                }
                break;

                default:
                    break;
            }
        }

        pos += length;
    }
}

/**
 * @brief tuya_os_adapt_bt 蓝牙广播包接收数据解析,用于产测
 * @return OPERATE_RET
 */
static void tuya_bt_assign_scan_decode(recv_adv_t *recv_adv)
{
    char name[32] = {0};

    if (ble_scan_found == TRUE || ble_scan_info == NULL) {
        return;
    }

    if (TY_BT_SCAN_BY_MAC == ble_scan_info->scan_type) {
        if (0 == memcmp(ble_scan_info->bd_addr, recv_adv->adv_addr, 6)) {
            ble_scan_found = TRUE;
        } else {
            return;
        }
    }

    if (tuya_bt_get_scan_local_name(recv_adv, (char *)name) != OPRT_OS_ADAPTER_OK) {
        return;
    }

    if (TY_BT_SCAN_BY_NAME == ble_scan_info->scan_type) {
        if (0 == strcmp(name, ble_scan_info->name)) {
            ble_scan_found = TRUE;
        } else {
            return;
        }
    }

    if (ble_scan_found) {
        memcpy(ble_scan_info->bd_addr, recv_adv->adv_addr, 6);
        strcpy(ble_scan_info->name, name);
        ble_scan_info->channel = 0;
        ble_scan_info->rssi = recv_adv->rssi;
    }


}

/**
 * @brief tuya_os_adapt_bt 蓝牙广播包接收cb
 * @return OPERATE_RET
 */
static void ble_recv_adv_callback(recv_adv_t *recv_adv)
{

    if (TY_BT_SCAN_BY_ADV != ble_scan_info->scan_type) { //正常数据

        tuya_bt_assign_scan_decode(recv_adv);

    } else {

        tuya_bt_scan_pre_decode(recv_adv);
    }
}

/**
 * @brief ble异常之后，重启ble
 *
 * @param[in] none
 * @return  OPRT_OS_ADAPTER_OK
 */
static void ty_ble_restart_thread(void *parameter)
{
    int ret = 0;

    while (1) {
        ret = tuya_os_adapt_semaphore_wait(ble_err_sem);
        if (ret != OPRT_OS_ADAPTER_OK) {
            LOG_ERR("ble wait sem error\r\n");
            return ;
        }

        tuya_os_adapt_system_sleep(100);
        ble_set_write_cb(ble_write_callback);
        ble_set_read_cb(ble_read_callback);
        ble_set_event_cb(ble_event_callback);
        ble_set_recv_adv_cb(ble_recv_adv_callback);
        ble_activate(NULL);
        
        tuya_os_adapt_system_sleep(500);
        
        //该模式适用于BK7231T下需要手工disconnect bt
        if (if_start_adv_after_disconnect) {
            tuya_os_adapt_bt_start_adv();
        } else {        //下次自动开启广播
            if_start_adv_after_disconnect = TRUE;
        }
        
        if (ble_scan_info) {
            ble_set_rf_time(70, 30);    //设置rf使用时间
            if (ble_scan_flag && (ERR_SUCCESS == appm_ll_scan_start())) {
                LOG_DEBUG("ble restart goto scan\r\n");
            } else {
                
            }
        }
        
        break;
    }

    tuya_os_adapt_semaphore_release(ble_err_sem);
    tuya_os_adapt_thread_release(ble_restart_thrd);

}


/**
 * @brief tuya_os_adapt_bt 蓝牙初始化
 * @return OPERATE_RET
 */
int tuya_os_adapt_bt_port_init(ty_bt_param_t *p)
{

    LOG_DEBUG("!!!!!!!!!!tuya_os_adapt_bt_port_init\r\n");

    if ((p != NULL) && (p->cb != NULL)) {
        ty_bt_msg_cb = p->cb;
    } else {
        LOG_ERR("!!!!!!!!!!tuya_os_adapt_bt_port_init, p is null\r\n");
    }

    ble_activate(NULL);
    ble_set_write_cb(ble_write_callback);
    ble_set_read_cb(ble_read_callback);
    ble_set_event_cb(ble_event_callback);
    ble_set_recv_adv_cb(ble_recv_adv_callback);


    return OPRT_OS_ADAPTER_OK;
}

/**
 * @brief tuya_os_adapt_bt 蓝牙断开关闭
 * @return OPERATE_RET
 */
int tuya_os_adapt_bt_port_deinit(void)
{
    unsigned char app_status = 0;
    extern uint8_t appm_get_app_status(void);
    app_status = appm_get_app_status();
    LOG_DEBUG("!!!!!!!!!!tuya_os_adapt_bt_port_deinit status:%d\r\n", app_status);

    if (APPM_ADVERTISING == app_status) {
        appm_stop_advertising();
    }
    if (APPM_CONNECTED == app_status) {
        if_start_adv_after_disconnect = false;
        appm_disconnect(0x13);
    }

    if (ty_bt_msg_cb) {
        ty_bt_msg_cb = NULL;
    }

    return OPRT_OS_ADAPTER_OK;
}

/**
 * @brief tuya_os_adapt_bt 蓝牙断开
 * @return OPERATE_RET
 */
int tuya_os_adapt_bt_gap_disconnect(void)
{
    LOG_DEBUG("!!!!!!!!!!tuya_os_adapt_bt_gap_disconnect\r\n");

    if_start_adv_after_disconnect = TRUE;
    appm_disconnect(0x13);

    return OPRT_OS_ADAPTER_OK;
}

/**
 * @brief tuya_os_adapt_bt 蓝牙发送
 * @return OPERATE_RET
 */
int tuya_os_adapt_bt_send(unsigned char *data, unsigned char len)
{
    os_printf("!!!!!!!!!!tuya_os_adapt_bt_send\r\n");
    while (ble_att_flag == 1) {
        rtos_delay_milliseconds(1);
    }
    ble_att_flag = 1;

    bk_ble_send_ntf_value(len, data, 0, 4);

    return OPRT_OS_ADAPTER_OK;
}

/**
 * @brief tuya_os_adapt_bt 广播包重置
 * @return OPERATE_RET
 */
int tuya_os_adapt_bt_reset_adv(tuya_ble_data_buf_t *adv, tuya_ble_data_buf_t *scan_resp)
{
    LOG_DEBUG("!!!!!!!!!!tuya_os_adapt_bt_reset_adv\r\n");

    memset(&adv_info, 0x00, sizeof(adv_info));

    adv_info.channel_map = 7;
    adv_info.interval_min = 160;
    adv_info.interval_max = 160;

    memcpy(adv_info.advData, adv->data, adv->len);
    adv_info.advDataLen = adv->len;

    memcpy(adv_info.respData, scan_resp->data, scan_resp->len);
    adv_info.respDataLen = scan_resp->len;

    if (ble_stack_init_ok) {
        appm_update_adv_data(adv->data, adv->len, scan_resp->data, scan_resp->len);
    }
    
    return OPRT_OS_ADAPTER_OK;
}

/**
 * @brief tuya_os_adapt_bt 获取rssi信号值
 * @return OPERATE_RET
 */
int tuya_os_adapt_bt_get_rssi(signed char *rssi)
{

    return OPRT_OS_ADAPTER_OK; // 暂时未实现
}

/**
 * @brief tuya_os_adapt_bt 停止广播
 * @return OPERATE_RET
 */
int tuya_os_adapt_bt_start_adv(void)
{
    LOG_DEBUG("!!!!!!!!!!tuya_os_adapt_bt_start_advn\r\n");
    if (OPRT_OS_ADAPTER_OK == appm_start_advertising()) {
        return OPRT_OS_ADAPTER_OK;

    }

    return OPRT_OS_ADAPTER_BT_ADV_START_FAILED;
}

extern uint8_t appm_get_app_status(void);
/**
 * @brief tuya_os_adapt_bt 停止广播
 * @return OPERATE_RET
 */
int tuya_os_adapt_bt_stop_adv(void)
{
    unsigned char app_status;
    
    LOG_DEBUG("!!!!!!!!!!tuya_os_adapt_bt_start_advn\r\n");
    app_status = appm_get_app_status();
    if (APPM_ADVERTISING == app_status) {
        if (appm_stop_advertising() != OPRT_OS_ADAPTER_OK) {
            LOG_ERR("!!!!!!!!!!tuya_os_adapt_bt_stop_adv stop err\r\n");
            return OPRT_OS_ADAPTER_BT_ADV_STOP_FAILED;
        }
    }

    return OPRT_OS_ADAPTER_OK;

}

/**
 * @brief tuya_os_adapt_bt 主动扫描蓝牙广播包
 * @return OPERATE_RET
 */
int tuya_os_adapt_bt_assign_scan(IN OUT ty_bt_scan_info_t *info)
{
    UINT_T timeout_count = 0;
    OPERATE_RET op_ret = OPRT_OS_ADAPTER_OK;

    LOG_DEBUG("tuya_bt_assign_scan(), start\r\n");

    if (!ble_scan_info) {
        ble_scan_info = (ty_bt_scan_info_t *)tuya_os_adapt_system_malloc(sizeof(ty_bt_scan_info_t));
        if (NULL == ble_scan_info) {
            op_ret = OPRT_OS_ADAPTER_MALLOC_FAILED;
            goto ERR_END;
        }
    }

    memcpy(ble_scan_info, info, sizeof(ty_bt_scan_info_t));  // 设置参数
    ble_scan_found = FALSE;

    if (ERR_SUCCESS != appm_ll_scan_start()) {
        return OPRT_OS_ADAPTER_COM_ERROR;
    }

    timeout_count = (info->timeout_s * 1000) / SCAN_SLEEP_MS;
    while (timeout_count > 0 && (!ble_scan_found)) {
        timeout_count--;
        tuya_os_adapt_system_sleep(SCAN_SLEEP_MS);
    }

    //bk_printf("ble_scan_found:%d, rssi:%d\r\n", ble_scan_found, ble_scan_info->rssi);
    if (ble_scan_found) {
        memcpy(info, ble_scan_info, sizeof(ty_bt_scan_info_t));
        op_ret = OPRT_OS_ADAPTER_OK;
    } else {
        op_ret = OPRT_OS_ADAPTER_COM_ERROR;
    }

    return op_ret;

ERR_END:
    appm_ll_scan_stop();
    if (ble_scan_found) {
        tuya_os_adapt_system_free(ble_scan_info);
        ble_scan_info = NULL;
    }

    LOG_DEBUG("tuya_bt_assign_scan(), end\r\n");

    return OPRT_OS_ADAPTER_COM_ERROR;
}

/**
 * @brief tuya_os_adapt_bt 广播接收初始化,包括监控数据状态和接收数据函数（用于蓝牙遥控器ffc） 老基线函数tuya_bt_ffc_regist
 * @return OPERATE_RET
 */
OPERATE_RET tuya_os_adapt_bt_scan_init(IN TY_BT_SCAN_ADV_CB scan_adv_cb)
{
    if (!ble_scan_info) {
        ble_scan_info = (ty_bt_scan_info_t *)tuya_os_adapt_system_malloc(sizeof(ty_bt_scan_info_t));
        if (NULL == ble_scan_info) {
            return OPRT_OS_ADAPTER_MALLOC_FAILED;
        }

        memset(ble_scan_info, 0x00, sizeof(ty_bt_scan_info_t));
    }

    if (ble_scan_found) {
        LOG_ERR("<tuya_os_adapt_bt_scan_init> ble scan found.\r\n");
        return OPRT_OS_ADAPTER_COM_ERROR;
    }

    ble_scan_info->scan_type = TY_BT_SCAN_BY_ADV;
    ble_scan_info->scan_adv_cb = scan_adv_cb;

    //设置蓝牙和WiFi的占空比，单位50ms
    ble_set_rf_time(70, 30);    //设置rf使用时间

    return OPRT_OS_ADAPTER_OK;
}

/**
 * @brief tuya_os_adapt_bt 广播接收scan start
 * @return OPERATE_RET
 */
OPERATE_RET tuya_os_adapt_bt_start_scan(void)
{
    LOG_DEBUG("<tuya_os_adapt_bt_start_scan> %d\r\n",ble_scan_info->scan_type);
    if (ble_scan_info && (TY_BT_SCAN_BY_ADV == ble_scan_info->scan_type)) {
        LOG_DEBUG("<tuya_os_adapt_bt_start_scan> !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n");
        if (ERR_SUCCESS == appm_ll_scan_start()) {
            ble_scan_flag = 1;
            LOG_DEBUG("<tuya_os_adapt_bt_start_scan> start scan adv.\r\n");
            return OPRT_OS_ADAPTER_OK;
        }
    }

    LOG_DEBUG("<tuya_os_adapt_bt_start_scan> err\r\n");
    return OPRT_OS_ADAPTER_BT_ADV_START_FAILED;
}

/**
 * @brief tuya_os_adapt_bt 广播接收scan stop
 * @return OPERATE_RET
 */
OPERATE_RET tuya_os_adapt_bt_stop_scan(void)
{
    LOG_DEBUG("<tuya_os_adapt_bt_stop_scan> %d\r\n",ble_scan_info->scan_type);
    if (ble_scan_info && (TY_BT_SCAN_BY_ADV == ble_scan_info->scan_type)) {
        if (ERR_SUCCESS == appm_ll_scan_stop()) {
            LOG_DEBUG("<tuya_os_adapt_bt_stop_scan> stop scan adv.\r\n");
            return OPRT_OS_ADAPTER_OK;
        }
    }

    LOG_DEBUG("<tuya_os_adapt_bt_stop_scan> err\r\n");
    return OPRT_OS_ADAPTER_BT_ADV_STOP_FAILED;
}

/**
 * @brief tuya_os_adapt_bt 接口注册
 * @return OPERATE_RET
 */
int tuya_os_adapt_reg_bt_intf(void)
{
    return tuya_os_adapt_reg_intf(INTF_BT, (void *)&m_tuya_os_bt_intfs);
}

#endif
