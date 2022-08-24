#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "uni_md5.h"
#include "aes_inf.h"
#include "rwip_config.h"
#include "ble_api.h"
#include "ble_pub.h"
#include "uart_pub.h"
#include "app_task.h"
#include "mcu_ps_pub.h"
#include "application.h"
#include "tuya_ble_type.h"
#include "tuya_os_adapt_bt.h"
#include "tuya_os_adapt_wifi.h"
#include "uni_log.h"
#include "tuya_os_adapter.h"
#include "tuya_os_adapter_error_code.h"

#if defined(TY_BT_MOD) && TY_BT_MOD == 1

#define BLE_OPEN         1

unsigned short ntf_enable;

bk_attm_desc_t btl_att_db[6] =
{
    //  Service Declaration
    [0]        =   {0x1910,  BK_PERM_SET(RD, ENABLE), 0, 0},

    //  Level Characteristic Declaration
    [1]        =   {0x2803,   BK_PERM_SET(RD, ENABLE), 0, 0},
    //  Level Characteristic Value
    //[2]        =   {0x2b11,   BK_PERM_SET(WRITE_REQ, ENABLE), BK_PERM_SET(RI, ENABLE) , 20},
    [2]        =   {0x2b11,   BK_PERM_SET(WRITE_REQ, ENABLE)|BK_PERM_SET(WRITE_COMMAND, ENABLE), BK_PERM_SET(RI, ENABLE) , 20},
    [3]        =   {0x2803,   BK_PERM_SET(RD, ENABLE), 0, 0},
    //  Level Characteristic Value
    [4]        =   {0x2b10,   BK_PERM_SET(NTF, ENABLE) , BK_PERM_SET(RI, ENABLE) , 20},

    //  Level Characteristic - Client Characteristic Configuration Descriptor

    [5]        =   {0x2902,  BK_PERM_SET(RD, ENABLE)|BK_PERM_SET(WRITE_REQ, ENABLE), 0, 0},
};

TY_BT_MSG_CB ty_bt_msg_cb;


unsigned char ble_att_flag = 0;
bool if_start_adv_after_disconnect = true;

/* add begin: by sunkz, interface regist */
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
    .scan_init      = NULL,
    .start_scan     = NULL,
    .stop_scan      = NULL,
};
/* add end */


/**
 * @brief tuya_os_adapt_bt 蓝牙写cb
 * @return none
 */
static void ble_write_callback(write_req_t *write_req)
{
    tuya_ble_data_buf_t data;

    if(write_req->att_idx == 5)
    {
        ntf_enable = (write_req->value[0]) | (write_req->value[1] << 8);
    }
    else
    {
        data.data = write_req->value;
        data.len = write_req->len;

        os_printf("%s len:%d\r\n", __FUNCTION__, data.len);
        if(ty_bt_msg_cb!=NULL) {
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
    if(read_req->att_idx == 5)
    {
        read_req->value = ntf_enable;
    }
    return 2;
}

/**
 * @brief tuya_os_adapt_bt 蓝牙事件cb
 * @return none
 */
static void ble_event_callback(ble_event_t event, void *param)
{
    switch(event)
    {
        case BLE_STACK_OK:
        {
            bk_printf("STACK INIT OK\r\n");
            struct bk_ble_db_cfg ble_db_cfg;
            ble_db_cfg.att_db = btl_att_db;
            ble_db_cfg.att_db_nb = 6;
            ble_db_cfg.prf_task_id = 0;
            ble_db_cfg.start_hdl = 0;
            ble_db_cfg.svc_perm = 0;
            ble_db_cfg.uuid = 0x1910;

            bk_ble_create_db(&ble_db_cfg);
        }
        break;
        case BLE_STACK_FAIL:
            bk_printf("STACK INIT FAIL\r\n");
        break;

        case BLE_CONNECT:
            bk_printf("BLE CONNECT\r\n");

            if(((struct gapc_connection_req_ind *)param)->sup_to < 200)
            {
                struct gapc_conn_param conn_param;
                conn_param.intv_max = ((struct gapc_connection_req_ind *)param)->con_interval;
                conn_param.intv_min = ((struct gapc_connection_req_ind *)param)->con_interval;
                conn_param.latency = ((struct gapc_connection_req_ind *)param)->con_latency;
                conn_param.time_out = 600;
                appm_update_param(&conn_param);
            }

            mcu_prevent_set(MCU_PS_BLE_FROBID);
            if(ty_bt_msg_cb!=NULL)
                ty_bt_msg_cb(0, TY_BT_EVENT_CONNECTED ,NULL);
        break;

        case BLE_DISCONNECT:
        {
            bk_printf("BLE DISCONNECT\r\n");
            ble_att_flag = 0;
            mcu_prevent_clear(MCU_PS_BLE_FROBID);
            if(ty_bt_msg_cb!=NULL)
                ty_bt_msg_cb(0, TY_BT_EVENT_DISCONNECTED ,NULL);
            
            //该模式适用于BK7231T下需要手工disconnect bt
            if(if_start_adv_after_disconnect)
                tuya_os_adapt_bt_start_adv();
            else        //下次自动开启广播
                if_start_adv_after_disconnect = TRUE;
        }
        break;

        case BLE_MTU_CHANGE:
            bk_printf("BLE_MTU_CHANGE:%d\r\n", *(unsigned short *)param);
        break;
        case BLE_CFG_NOTIFY:
            bk_printf("BLE_CFG_NOTIFY:%d\r\n", *(unsigned short *)param);
        break;
        case BLE_CFG_INDICATE:
            bk_printf("BLE_CFG_INDICATE:%d\r\n", *(unsigned short *)param);
        break;
        case BLE_TX_DONE:
            os_printf("BLE_TX_DONE\r\n");
            ble_att_flag = 0;
        break;
        case BLE_GEN_DH_KEY:
        {
            bk_printf("BLE_GEN_DH_KEY\r\n");
            bk_printf("key_len:%d\r\n", ((struct ble_gen_dh_key_ind *)param)->len);
            for(int i = 0; i < ((struct ble_gen_dh_key_ind *)param)->len; i++)
            {
                bk_printf("%02x ", ((struct ble_gen_dh_key_ind *)param)->result[i]);
            }
            bk_printf("\r\n");
        }    
        break;
        case BLE_GET_KEY:
        {
            bk_printf("BLE_GET_KEY\r\n");
            bk_printf("pri_len:%d\r\n", ((struct ble_get_key_ind *)param)->pri_len);
            for(int i = 0; i < ((struct ble_get_key_ind *)param)->pri_len; i++)
            {
                bk_printf("%02x ", ((struct ble_get_key_ind *)param)->pri_key[i]);
            }
            bk_printf("\r\n");
        }    
        break;
        case BLE_CREATE_DB_OK:
        {
            bk_printf("CREATE DB SUCCESS\r\n");

            if(ty_bt_msg_cb!=NULL)
                ty_bt_msg_cb(0, TY_BT_EVENT_ADV_READY ,NULL);
            
            bk_printf("!!!!!!!!!!tuya_before_netcfg_cb\r\n");
            appm_start_advertising();
        }
        break;
        default:
            bk_printf("UNKNOW EVENT\r\n");
        break;
    }
}

/**
 * @brief tuya_os_adapt_bt 蓝牙接收cb
 * @return OPERATE_RET 
 */
static void ble_recv_adv_callback(uint8_t *buf, uint8_t len)
{
#if (BLE_APP_CLIENT)
    uint8_t find = 0;

    find = appm_adv_data_decode(len, buf, NULL, 0);

    if(find)
    {
        extern int *scan_check_result;
        if(scan_check_result)
        {
            *scan_check_result = 2;
            bk_printf("scan_check_result\r\n");
        }
    }
#endif
}

/**
 * @brief tuya_os_adapt_bt 蓝牙初始化
 * @return OPERATE_RET 
 */
int tuya_os_adapt_bt_port_init(ty_bt_param_t *p)
{

    bk_printf("!!!!!!!!!!tuya_os_adapt_bt_port_init\r\n");

    if((p!=NULL)&&(p->cb!=NULL)) {
        ty_bt_msg_cb = p->cb;
    } else {
        bk_printf("!!!!!!!!!!tuya_os_adapt_bt_port_init, p is null\r\n");
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
    app_status = appm_get_app_status();
    bk_printf("!!!!!!!!!!tuya_os_adapt_bt_port_deinit status:%d\r\n", app_status);
    if(APPM_ADVERTISING == app_status) 
    {
        appm_stop_advertising();
    }
    if(APPM_CONNECTED == app_status)
    {
        if_start_adv_after_disconnect = false;
        appm_disconnect(0x13);
    }

    if(ty_bt_msg_cb) {
        tuya_os_adapt_system_free(ty_bt_msg_cb);
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
    bk_printf("!!!!!!!!!!tuya_os_adapt_bt_gap_disconnect\r\n");
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
    while(ble_att_flag == 1) {
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
    bk_printf("!!!!!!!!!!tuya_os_adapt_bt_reset_adv\r\n");

    memset(&adv_info, 0x00, sizeof(adv_info));

    adv_info.channel_map = 7;
    adv_info.interval_min = 160;
    adv_info.interval_max = 160;

    memcpy(adv_info.advData, adv->data, adv->len);
    adv_info.advDataLen = adv->len;

    memcpy(adv_info.respData, scan_resp->data, scan_resp->len);
    adv_info.respDataLen = scan_resp->len;

    appm_update_adv_data(adv->data, adv->len, scan_resp->data, scan_resp->len);

    return OPRT_OS_ADAPTER_OK;
}


/**
 * @brief tuya_os_adapt_bt 获取rssi信号值
 * @return OPERATE_RET 
 */
int tuya_os_adapt_bt_get_rssi(signed char *rssi)
{

    return OPRT_OS_ADAPTER_OK;
}

/**
 * @brief tuya_os_adapt_bt 停止广播
 * @return OPERATE_RET 
 */
int tuya_os_adapt_bt_start_adv(void)
{
    bk_printf("!!!!!!!!!!tuya_os_adapt_bt_start_advn\r\n");
    if(OPRT_OS_ADAPTER_OK == appm_start_advertising()) {
        return OPRT_OS_ADAPTER_OK;

    }

    return OPRT_OS_ADAPTER_BT_ADV_START_FAILED;
}

/**
 * @brief tuya_os_adapt_bt 停止广播
 * @return OPERATE_RET 
 */
int tuya_os_adapt_bt_stop_adv(void)
{
    unsigned char app_status;

    app_status = appm_get_app_status();
    if(APPM_ADVERTISING == app_status)
    {
        if(appm_stop_advertising() != OPRT_OS_ADAPTER_OK) {
            bk_printf("!!!!!!!!!!tuya_os_adapt_bt_stop_adv stop err\r\n");
            return OPRT_OS_ADAPTER_BT_ADV_STOP_FAILED;
        }
    }
    if(APPM_CONNECTED == app_status)
    {
        if_start_adv_after_disconnect = false;
        appm_disconnect(0x13);
        bk_printf("!!!!!!!!!!tuya_os_adapt_bt_stop_adv 0x13 \r\n");
    }
    
    return OPRT_OS_ADAPTER_OK;

}

/**
 * @brief tuya_os_adapt_bt 主动扫描
 * @return OPERATE_RET 
 */
int tuya_os_adapt_bt_assign_scan(IN OUT ty_bt_scan_info_t *info)
{
    return OPRT_OS_ADAPTER_OK;
}

/**
 * @brief tuya_os_adapt_bt 接口注册
 * @return OPERATE_RET 
 */
int tuya_os_adapt_reg_bt_intf(void)
{
    return tuya_os_adapt_reg_intf(INTF_BT, &m_tuya_os_bt_intfs);
}
/* add end */

#endif