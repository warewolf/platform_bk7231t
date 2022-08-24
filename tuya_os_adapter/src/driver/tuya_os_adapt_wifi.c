/**
 * @file tuya_os_adapt_wifi.c
 * @brief wifi操作接口
 *
 * @copyright Copyright(C),2018-2020, 涂鸦科技 www.tuya.com
 *
 */

#include "tuya_os_adapt_wifi.h"
#include "tuya_os_adapter.h"
#include "tuya_os_adapt_output.h"
#include "tuya_os_adapt_semaphore.h"
#include "tuya_os_adapt_thread.h"
#include "tuya_os_adapt_queue.h"
#include "tuya_os_adapt_memory.h"


#include "rw_pub.h"
#include "wlan_ui_pub.h"
#include "uart_pub.h"
#include "ieee802_11_defs.h"
#include "rw_pub.h"


/***********************************************************
*************************micro define***********************
***********************************************************/

#define SCAN_MAX_AP 64

#define UNVALID_SIGNAL -127
#define SCAN_WITH_SSID 1

#define WIFI_MGNT_FRAME_RX_MSG              (1 << 0)
#define WIFI_MGNT_FRAME_TX_MSG              (1 << 1)


/***********************************************************
*************************variable define********************
***********************************************************/

typedef struct {
    AP_IF_S *ap_if;
    uint8_t ap_if_nums;
    uint8_t ap_if_count;
    SEM_HANDLE sem_handle;
} SACN_AP_RESULT_S;

typedef struct {
    unsigned char msg_id;
    unsigned int len;
    unsigned char *data;
} wifi_mgnt_frame;

static WIFI_REV_MGNT_CB mgnt_recv_cb = NULL;

wifi_country_t country_code[] = {
    {.cc = "CN", .schan = 1, .nchan = 13, .max_tx_power = 0, .policy = WIFI_COUNTRY_POLICY_MANUAL},
    {.cc = "US", .schan = 1, .nchan = 11, .max_tx_power = 0, .policy = WIFI_COUNTRY_POLICY_MANUAL},
    {.cc = "JP", .schan = 1, .nchan = 14, .max_tx_power = 0, .policy = WIFI_COUNTRY_POLICY_MANUAL},
    {.cc = "EU", .schan = 1, .nchan = 13, .max_tx_power = 0, .policy = WIFI_COUNTRY_POLICY_MANUAL},
    //{.cc= "AU", .schan=1, .nchan=13, .max_tx_power=0, .policy=WIFI_COUNTRY_POLICY_MANUAL}
};


static WF_WK_MD_E wf_mode = WWM_STATION; //WWM_LOWPOWER
static SNIFFER_CALLBACK snif_cb = NULL;

static SEM_HANDLE scanHandle = NULL;

static bool lp_mode = FALSE;

static unsigned char lp_status = 1; //不管应用是否设置，都是默认低功耗，dtim = 1

static const TUYA_OS_WIFI_INTF m_tuya_os_wifi_intfs = {
    .all_ap_scan                  = tuya_os_adapt_wifi_all_ap_scan,
    .assign_ap_scan               = tuya_os_adapt_wifi_assign_ap_scan,
    .release_ap                   = tuya_os_adapt_wifi_release_ap,
    .set_cur_channel              = tuya_os_adapt_wifi_set_cur_channel,
    .get_cur_channel              = tuya_os_adapt_wifi_get_cur_channel,
    .sniffer_set                  = tuya_os_adapt_wifi_sniffer_set,
    .get_ip                       = tuya_os_adapt_wifi_get_ip,
    .set_mac                      = tuya_os_adapt_wifi_set_mac,
    .get_mac                      = tuya_os_adapt_wifi_get_mac,
    .set_work_mode                = tuya_os_adapt_wifi_set_work_mode,
    .get_work_mode                = tuya_os_adapt_wifi_get_work_mode,
    .ap_start                     = tuya_os_adapt_wifi_ap_start,
    .ap_stop                      = tuya_os_adapt_wifi_ap_stop,
    .get_connected_ap_info_v2     = NULL,
    .fast_station_connect_v2      = NULL,
    .station_connect              = tuya_os_adapt_wifi_station_connect,
    .station_disconnect           = tuya_os_adapt_wifi_station_disconnect,
    .station_get_conn_ap_rssi     = tuya_os_adapt_wifi_station_get_conn_ap_rssi,
    .get_bssid                    = tuya_os_adapt_wifi_get_bssid,
    .station_get_status           = tuya_os_adapt_wifi_station_get_status,
    .set_country_code             = tuya_os_adapt_wifi_set_country_code,
    .send_mgnt                    = tuya_os_adapt_wifi_send_mgnt,
    .register_recv_mgnt_callback  = tuya_os_adapt_wifi_register_recv_mgnt_callback,
    .set_lp_mode                  = tuya_os_adapt_set_wifi_lp_mode,
    .rf_calibrated                = tuya_os_adapt_wifi_rf_calibrated,
};

/***********************************************************
*************************function define********************
***********************************************************/
static void ty_wifi_powersave_disable(void);

/**
 * @brief 原厂 scan 结束的CB
 *
 * @param[out]
 * @return  OPRT_OS_ADAPTER_OK: success  Other: fail
 */
static void scan_cb(void *ctxt, unsigned char param)
{
    if (scanHandle) {
        tuya_os_adapt_semaphore_post(scanHandle);
    }
}

/**
 * @brief scan current environment and obtain all the ap
 *        infos in current environment
 *
 * @param[out]      ap_ary      current ap info array
 * @param[out]      num         the num of ar_ary
 * @return  OPRT_OS_ADAPTER_OK: success  Other: fail
 */
int tuya_os_adapt_wifi_all_ap_scan(AP_IF_S **ap_ary, unsigned int *num)
{
    AP_IF_S *item;
    AP_IF_S *array = NULL;
    OPERATE_RET ret;
    INT_T i;
    INT_T scan_cnt;
    INT_T ssid_len;
    ScanResult_adv apList;

    if ((NULL == ap_ary) || (NULL == num)) {
        return OPRT_OS_ADAPTER_INVALID_PARM;
    }

    ret = tuya_os_adapt_semaphore_create_init(&scanHandle, 0, 1);
    if (ret !=  OPRT_OS_ADAPTER_OK) {
        return ret;
    }

    mhdr_scanu_reg_cb(scan_cb, 0);
    bk_wlan_start_scan();

    ret = tuya_os_adapt_semaphore_wait(scanHandle);
    if (ret !=  OPRT_OS_ADAPTER_OK) {
        LOG_ERR("scan wait sem error\r\n");
        return ret;
    }

    tuya_os_adapt_semaphore_release(scanHandle);
    scanHandle = NULL;

    if ((wlan_sta_scan_result(&apList) != 0) || (0 == apList.ApNum)) {
        LOG_ERR("scan error\r\n");
        goto SCAN_ERR;
    }

    scan_cnt = apList.ApNum;

    if (scan_cnt > SCAN_MAX_AP) {
        scan_cnt = SCAN_MAX_AP;
    }

    array = (AP_IF_S *)tuya_os_adapt_system_malloc(SIZEOF(AP_IF_S) * scan_cnt);
    if (NULL == array) {
        goto SCAN_ERR;
    }

    memset(array, 0, (sizeof(AP_IF_S) * scan_cnt));
    for (i = 0; i < scan_cnt; i++) {
        item = &array[i];

        item->channel = apList.ApList[i].channel;
        item->rssi = apList.ApList[i].ApPower;
        os_memcpy(item->bssid, apList.ApList[i].bssid, 6);

        ssid_len = os_strlen(apList.ApList[i].ssid);
        if (ssid_len > WIFI_SSID_LEN) {
            ssid_len = WIFI_SSID_LEN;
        }
        os_strncpy((char *)item->ssid, apList.ApList[i].ssid, ssid_len);
        item->s_len = ssid_len;
    }

    *ap_ary = array;
    *num = scan_cnt & 0xff;
    if (apList.ApList != NULL) {
        tuya_os_adapt_system_free(apList.ApList);
    }
    return  OPRT_OS_ADAPTER_OK;

SCAN_ERR:
    if (apList.ApList != NULL) {
        tuya_os_adapt_system_free(apList.ApList);
        apList.ApList = NULL;
    }

    if (array) {
        tuya_os_adapt_system_free(array);
        array = NULL;
    }
    return OPRT_OS_ADAPTER_COM_ERROR;

}

/*
 * @param[in]       ssid        the specific ssid
 * @param[out]      ap          the ap info
 * @return  OPRT_OS_ADAPTER_OK: success  Other: fail
 */
int tuya_os_adapt_wifi_assign_ap_scan(const signed char *ssid, AP_IF_S **ap)
{
    AP_IF_S *array = NULL;
    OPERATE_RET ret;
    INT_T i = 0, j = 0, ssid_len;
    ScanResult_adv apList;

    if ((NULL == ssid) || (NULL == ap)) {
        return OPRT_INVALID_PARM;
    }

    memset(&apList, 0, sizeof(ScanResult_adv));

    ret = tuya_os_adapt_semaphore_create_init(&scanHandle, 0, 1);
    if (ret != OPRT_OK) {
        return ret;
    }

    mhdr_scanu_reg_cb(scan_cb, 0);
    bk_wlan_start_assign_scan((UINT8 **)&ssid, 1);

    ret = tuya_os_adapt_semaphore_wait(scanHandle);
    if (ret != OPRT_OS_ADAPTER_OK) {
        LOG_ERR("scan wait sem error\r\n");
        return ret;
    }

    tuya_os_adapt_semaphore_release(scanHandle);
    scanHandle = NULL;

    if ((wlan_sta_scan_result(&apList) != 0) || (0 == apList.ApNum)) {
        LOG_ERR("scan error\r\n");
        goto SCAN_ERR;
    }

    array = (AP_IF_S *)tuya_os_adapt_system_malloc(sizeof(AP_IF_S));
    if (NULL == array) {
        goto SCAN_ERR;
    }

    memset(array, 0, sizeof(AP_IF_S));
    array->rssi = -100;

    /* iterate scan result list to find specified ssid */
    for (i = 0; i < apList.ApNum ; i++) {
        /* skip non-matched ssid */
        if (os_strcmp(apList.ApList[i].ssid, (char *)ssid))
            continue;

        /* found */
        if (apList.ApList[i].ApPower < array->rssi) { /* rssi 信号强度比较 */
            continue;
        }
        array->channel = apList.ApList[i].channel;
        array->rssi = apList.ApList[i].ApPower;
        os_memcpy(array->bssid, apList.ApList[i].bssid, 6);

        ssid_len = os_strlen(apList.ApList[i].ssid);
        if (ssid_len > WIFI_SSID_LEN) {
            ssid_len = WIFI_SSID_LEN;
        }
        os_strncpy((char *)array->ssid, apList.ApList[i].ssid, ssid_len);
        array->s_len = ssid_len;

        j++;
    }

    if (j == 0) {
        goto SCAN_ERR;
    }

    *ap = array;
    if (apList.ApList != NULL) {
        tuya_os_adapt_system_free(apList.ApList);
    }

    return OPRT_OS_ADAPTER_OK;

SCAN_ERR:
    if (apList.ApList != NULL) {
        tuya_os_adapt_system_free(apList.ApList);
        apList.ApList = NULL;
    }

    if (array) {
        tuya_os_adapt_system_free(array);
        array = NULL;
    }
    return OPRT_OS_ADAPTER_COM_ERROR;

}

/**
 * @brief release the memory malloced in <tuya_os_adapt_wifi_all_ap_scan>
 *        and <tuya_os_adapt_wifi_assign_ap_scan> if needed. tuya-sdk
 *        will call this function when the ap info is no use.
 *
 * @param[in]       ap          the ap info
 * @return  OPRT_OS_ADAPTER_OK: success  Other: fail
 */
int tuya_os_adapt_wifi_release_ap(AP_IF_S *ap)
{
    if (NULL == ap) {
        return OPRT_OS_ADAPTER_INVALID_PARM;
    }

    tuya_os_adapt_system_free(ap);
    ap = NULL;
    
    return OPRT_OS_ADAPTER_OK;
}

/**
 * @brief wifi 管理祯回调，注册到原厂
 *
 * @param[in] frame len *param
 * @return none
 */
static VOID wifi_mgnt_frame_rx_notify(const uint8_t *frame, int len, void *param)
{
#if 0 //debuge
    WLAN_FRAME_S *buf = (WLAN_FRAME_S *)frame;
    char ssid_test[33] = {0};
    
    if (WFT_BEACON == buf->frame_type) {
       
        //AP_IF_S ap_info = {0};
        strncpy(ssid_test, buf->frame_data.beacon_info.ssid, buf->frame_data.beacon_info.ssid_len);

        ssid_test[buf->frame_data.beacon_info.ssid_len] = '\0';
        bk_printf("ssid %s\r\n", ssid_test);
        
    }
#endif

    if(mgnt_recv_cb) {
         mgnt_recv_cb((unsigned char *)frame, len);
    }
}

/**
 * @brief set wifi interface work channel
 *
 * @param[in]       chan        the channel to set
 * @return  OPRT_OS_ADAPTER_OK: success  Other: fail
 */
int tuya_os_adapt_wifi_set_cur_channel(const unsigned char chan)
{
    int status;
        
    if ((chan > 14) || (chan < 1)) {
        return OPRT_OS_ADAPTER_INVALID_PARM;
    }
    
    if (mgnt_recv_cb != NULL) { /* 为了避免路由器断开之后，管理祯接收NG的问题 */            
        LOG_DEBUG("mgnt recv cb set<<<<\r\n");
        if (HW_IDLE == nxmac_current_state_getf()) {
            ty_wifi_powersave_disable();     /* 强制退出低功耗的模式 */
            LOG_DEBUG("set mm active~~~~~\r\n");
            rw_msg_send_mm_active_req();//强制激活mac控制层
        }
        bk_wlan_reg_rx_mgmt_cb((mgmt_rx_cb_t)wifi_mgnt_frame_rx_notify, 2);    /* 将回调注册到原厂中 */
    }
    
    
    status = bk_wlan_set_channel(chan);
    if (status) {
        return OPRT_OS_ADAPTER_CHAN_SET_FAILED;
    } else {
        return OPRT_OS_ADAPTER_OK;
    }

}

/**
 * @brief get wifi interface work channel
 *
 * @param[out]      chan        the channel wifi works
 * @return  OPRT_OS_ADAPTER_OK: success  Other: fail
 */
int tuya_os_adapt_wifi_get_cur_channel(unsigned char *chan)
{
    *chan = (unsigned char)bk_wlan_get_channel();

    return OPRT_OS_ADAPTER_OK;
}

static void _wf_sniffer_set_cb(unsigned char *data, int len, hal_wifi_link_info_t *info)
{
    if (NULL != snif_cb) {
        (*snif_cb)(data, len, info->rssi);
    }
}

/**
 * @brief enable / disable wifi sniffer mode.
 *        if wifi sniffer mode is enabled, wifi recv from
 *        packages from the air, and user shoud send these
 *        packages to tuya-sdk with callback <cb>.
 *
 * @param[in]       en          enable or disable
 * @param[in]       cb          notify callback
 * @return  OPRT_OK: success  Other: fail
 */
int tuya_os_adapt_wifi_sniffer_set(const BOOL_T en, const SNIFFER_CALLBACK cb)
{
    if (en) {
        WF_WK_MD_E mode = WWM_LOWPOWER;
        tuya_os_adapt_wifi_get_work_mode(&mode);
        if ((mode == WWM_SOFTAP) || (mode == WWM_STATIONAP)) {
            bk_wlan_set_ap_monitor_coexist(1);
        } else {
            bk_wlan_set_ap_monitor_coexist(0);
        }

        snif_cb = cb;
        bk_wlan_register_monitor_cb(_wf_sniffer_set_cb);

        bk_wlan_start_monitor();

    } else {
        bk_wlan_register_monitor_cb(NULL);
        bk_wlan_stop_monitor();

    }
    return OPRT_OS_ADAPTER_OK;
}

/**
 * @brief get wifi ip info.when wifi works in
 *        ap+station mode, wifi has two ips.
 *
 * @param[in]       wf          wifi function type
 * @param[out]      ip          the ip addr info
 * @return  OPRT_OS_ADAPTER_OK: success  Other: fail
 */
int tuya_os_adapt_wifi_get_ip(const WF_IF_E wf, NW_IP_S *ip)
{
    int ret = OPRT_OS_ADAPTER_OK;
    IPStatusTypedef wNetConfig;
    WiFi_Interface iface;

    os_memset(&wNetConfig, 0x0, sizeof(IPStatusTypedef));

    switch (wf) {
        case WF_STATION:
            iface = STATION;
            wNetConfig.dhcp = DHCP_CLIENT;
            break;

        case WF_AP:
            iface = SOFT_AP;
            wNetConfig.dhcp = DHCP_SERVER;
            break;

        default:
            ret = OPRT_OS_ADAPTER_INVALID_PARM;
            break;
    }

    if (OPRT_OS_ADAPTER_OK == ret) {
        bk_wlan_get_ip_status(&wNetConfig, iface);
        os_strcpy(ip->ip, wNetConfig.ip);
        os_strcpy(ip->mask, wNetConfig.mask);
        os_strcpy(ip->gw, wNetConfig.gate);
    }

    return ret;
}

/**
 * @brief get wifi mac info.when wifi works in
 *        ap+station mode, wifi has two macs.
 *
 * @param[in]       wf          wifi function type
 * @param[out]      mac         the mac info
 * @return  OPRT_OS_ADAPTER_OK: success  Other: fail
 */
int tuya_os_adapt_wifi_get_mac(const WF_IF_E wf, NW_MAC_S *mac)
{
    if (WF_STATION == wf) {
        wifi_get_mac_address((char *)mac, 2);
    } else {
        wifi_get_mac_address((char *)mac, 1);
    }

    return OPRT_OS_ADAPTER_OK;
}

/**
 * @brief set wifi mac info.when wifi works in
 *        ap+station mode, wifi has two macs.
 *
 * @param[in]       wf          wifi function type
 * @param[in]       mac         the mac info
 * @return  OPRT_OS_ADAPTER_OK: success  Other: fail
 */
int tuya_os_adapt_wifi_set_mac(const WF_IF_E wf, const NW_MAC_S *mac)
{
    if (wifi_set_mac_address((char *)mac)) {
        return OPRT_OS_ADAPTER_OK;
    } else {
        return OPRT_OS_ADAPTER_MAC_SET_FAILED;
    }
}

static int _wf_wk_mode_exit(WF_WK_MD_E mode)
{
    int ret = OPRT_OS_ADAPTER_OK;

    switch (mode) {
        case WWM_LOWPOWER :
            break;

        case WWM_SNIFFER:
            bk_wlan_stop_monitor();
            break;

        case WWM_STATION:
            bk_wlan_stop(STATION);
            break;

        case WWM_SOFTAP:
            bk_wlan_stop(SOFT_AP);
            break;

        case WWM_STATIONAP:
            bk_wlan_stop(SOFT_AP);
            bk_wlan_stop(STATION);
            break;

        default:
            break;
    }

    return ret;
}

extern void extended_app_waiting_for_launch(void);
/**
 * @brief set wifi work mode
 *
 * @param[in]       mode        wifi work mode
 * @return  OPRT_OS_ADAPTER_OK: success  Other: fail
 */
int tuya_os_adapt_wifi_set_work_mode(const WF_WK_MD_E mode)
{
    OPERATE_RET ret = OPRT_OS_ADAPTER_OK;
    WF_WK_MD_E current_mode;
    static unsigned char first_set_flag = TRUE;

    if (first_set_flag) {
        extended_app_waiting_for_launch(); /* 首次等待wifi初始化OK */
        first_set_flag = FALSE;
    }


    ret = tuya_os_adapt_wifi_get_work_mode(&current_mode);
    if ((OPRT_OS_ADAPTER_OK == ret) && (current_mode != mode)) {
        _wf_wk_mode_exit(current_mode);
    }
    wf_mode = mode;

    return ret;
}

/**
 * @brief get wifi work mode
 *
 * @param[out]      mode        wifi work mode
 * @return  OPRT_OS_ADAPTER_OK: success  Other: fail
 */
int tuya_os_adapt_wifi_get_work_mode(WF_WK_MD_E *mode)
{
    *mode = wf_mode;

    return OPRT_OS_ADAPTER_OK;
}

/**
 * @brief connect wifi with ssid and passwd
 *
 * @param[in]       ssid
 * @param[in]       passwd
 * @attention only support wap/wap2
 * @return  OPRT_OS_ADAPTER_OK: success  Other: fail
 */
int tuya_os_adapt_wifi_station_connect(const signed char *ssid, const signed char *passwd)
{
    int ret = OPRT_OS_ADAPTER_OK;
    network_InitTypeDef_st wNetConfig;
    os_memset(&wNetConfig, 0x0, sizeof(network_InitTypeDef_st));

    os_strcpy((char *)wNetConfig.wifi_ssid, (const char *)ssid);
    os_strcpy((char *)wNetConfig.wifi_key, (const char *)passwd);

    wNetConfig.wifi_mode = STATION;
    wNetConfig.dhcp_mode = DHCP_CLIENT;

    ret = bk_wlan_start(&wNetConfig);
    if (OPRT_OS_ADAPTER_OK != ret) {
        LOG_ERR("wlan start err\r\n");
    }

    return OPRT_OS_ADAPTER_OK;
}

/**
 * @brief disconnect wifi from connect ap
 *
 * @return  OPRT_OS_ADAPTER_OK: success  Other: fail
 */
int tuya_os_adapt_wifi_station_disconnect(void)
{
    //bk_wlan_stop(STATION); /* 不需要实现，由于在start中一开始就会停止 */
    return OPRT_OS_ADAPTER_OK;
}

/**
 * @brief get wifi connect rssi
 *
 * @param[out]      rssi        the return rssi
 * @return  OPRT_OS_ADAPTER_OK: success  Other: fail
 */
int tuya_os_adapt_wifi_station_get_conn_ap_rssi(signed char *rssi)
{
    int ret = OPRT_OS_ADAPTER_OK;
    LinkStatusTypeDef sta;

    ret = bk_wlan_get_link_status(&sta);
    if (OPRT_OS_ADAPTER_OK == ret) {
        *rssi = sta.wifi_strength;
    }

    return ret;
}

/**
 * @brief get wifi bssid
 *
 * @param[out]      mac         uplink mac
 * @return  OPRT_OS_ADAPTER_OK: success  Other: fail
 */
int tuya_os_adapt_wifi_get_bssid(unsigned char *mac)
{
    int ret = OPRT_OS_ADAPTER_OK;
    LinkStatusTypeDef sta;

    if (NULL == mac) {
        return OPRT_OS_ADAPTER_INVALID_PARM;
    }

    ret = bk_wlan_get_link_status(&sta);
    if (OPRT_OS_ADAPTER_OK == ret) {
        os_memcpy(mac, sta.bssid, 6);
    }

    return ret;
}

extern int tuya_os_adapt_wifi_get_lp_mode(void);
/**
 * @brief get wifi station work status
 *
 * @param[out]      stat        the wifi station work status
 * @return  OPRT_OS_ADAPTER_OK: success  Other: fail
 */
int tuya_os_adapt_wifi_station_get_status(WF_STATION_STAT_E *stat)
{
    static unsigned char flag = FALSE;
    rw_evt_type type = mhdr_get_station_status();
    {
        switch (type) {
            case RW_EVT_STA_IDLE:
                *stat = WSS_IDLE;
                break;
            case RW_EVT_STA_CONNECTING:
                *stat = WSS_CONNECTING;
                break;
            case RW_EVT_STA_PASSWORD_WRONG:
                *stat = WSS_PASSWD_WRONG;
                break;
            case RW_EVT_STA_NO_AP_FOUND:
                *stat = WSS_NO_AP_FOUND;
                break;
            case RW_EVT_STA_ASSOC_FULL:
                *stat = WSS_CONN_FAIL;
                break;
            case RW_EVT_STA_BEACON_LOSE:
                *stat = WSS_CONN_FAIL;
                break;
            case RW_EVT_STA_DISCONNECTED:
                *stat = WSS_CONN_FAIL;
                break;
            case RW_EVT_STA_CONNECT_FAILED:
                *stat = WSS_CONN_FAIL;
                break;
            case RW_EVT_STA_CONNECTED:
                *stat = WSS_CONN_SUCCESS;
                break;
            case RW_EVT_STA_GOT_IP:
                *stat = WSS_GOT_IP;
                break;
            default:
                break;
        }
    }

    if ((!flag) && (*stat == WSS_GOT_IP)) {

        if (lp_status) {
            bk_wlan_dtim_rf_ps_mode_enable();
            bk_wlan_dtim_rf_ps_timer_start();
        }
        if (tuya_os_adapt_wifi_get_lp_mode()) {
            bk_wlan_mcu_ps_mode_enable();
            lp_mode = TRUE;
        } else {
            lp_mode = FALSE;
        }
        
        if (mgnt_recv_cb != NULL) { /* 为了避免路由器断开之后，管理祯接收NG的问题 */
            ty_wifi_powersave_disable();                              /* 强制退出低功耗的模式 */
            bk_wlan_reg_rx_mgmt_cb((mgmt_rx_cb_t)wifi_mgnt_frame_rx_notify, 2);    /* 将回调注册到原厂中 */
            if (HW_IDLE == nxmac_current_state_getf()) {
                rw_msg_send_mm_active_req();//强制激活mac控制层
            }
        }

        flag = TRUE;
    } else if (*stat != WSS_GOT_IP) {
        flag = FALSE;
        
    }

    return OPRT_OS_ADAPTER_OK;
}

/**
 * @brief start a soft ap
 *
 * @param[in]       cfg         the soft ap config
 * @return  OPRT_OS_ADAPTER_OK: success  Other: fail
 */
int tuya_os_adapt_wifi_ap_start(const WF_AP_CFG_IF_S *cfg)
{
    int ret = OPRT_OS_ADAPTER_OK;
    network_InitTypeDef_ap_st wNetConfig;

    os_memset(&wNetConfig, 0x0, sizeof(network_InitTypeDef_ap_st));

    switch (cfg->md) {
        case WAAM_OPEN :
            wNetConfig.security = SECURITY_TYPE_NONE;
            break;
        case WAAM_WEP :
            wNetConfig.security = SECURITY_TYPE_WEP;
            break;
        case WAAM_WPA_PSK :
            wNetConfig.security = SECURITY_TYPE_WPA2_TKIP;
            break;
        case WAAM_WPA2_PSK :
            wNetConfig.security = SECURITY_TYPE_WPA2_AES;
            break;
        case WAAM_WPA_WPA2_PSK:
            wNetConfig.security = SECURITY_TYPE_WPA2_MIXED;
            break;
        default:
            ret = OPRT_INVALID_PARM;
            break;
    }

    if (OPRT_OS_ADAPTER_OK == ret) {
        wNetConfig.channel = cfg->chan;
        wNetConfig.dhcp_mode = DHCP_SERVER;
        os_memcpy((char *)wNetConfig.wifi_ssid, cfg->ssid, cfg->s_len);
        os_memcpy((char *)wNetConfig.wifi_key, cfg->passwd, cfg->p_len);

        os_strcpy((char *)wNetConfig.local_ip_addr, "192.168.175.1");
        os_strcpy((char *)wNetConfig.net_mask, "255.255.255.0");
        os_strcpy((char *)wNetConfig.gateway_ip_addr, "192.168.175.1");
        os_strcpy((char *)wNetConfig.dns_server_ip_addr, "192.168.175.1");

        LOG_DEBUG("ssid:%s key:%s  channnel: %d\r\n", wNetConfig.wifi_ssid, wNetConfig.wifi_key, wNetConfig.channel);
        ret = bk_wlan_start_ap_adv(&wNetConfig);
        if (OPRT_OS_ADAPTER_OK != ret) {
            LOG_ERR("start ap err\r\n");
        }
    }

    return ret;
}

/**
 * @brief stop a soft ap
 *
 * @return  OPRT_OS_ADAPTER_OK: success  Other: fail
 */
int tuya_os_adapt_wifi_ap_stop(void)
{
    bk_wlan_stop(SOFT_AP);

    return OPRT_OS_ADAPTER_OK;
}

/**
 * @brief 快连信息输出给SDK，SDK实现快连
 *
 * @param[in] FAST_WF_CONNECTED_AP_INFO_V2_S *fast_ap_info
 * @return  OPRT_OS_ADAPTER_OK: success  Other: fail
 */
int tuya_os_adapt_wifi_get_connected_ap_info_v2(FAST_WF_CONNECTED_AP_INFO_V2_S **fast_ap_info)
{
    return OPRT_OS_ADAPTER_OK;
}

/**
 * @brief 快连信息SDK输出到开发环境中，开发环境负责保存
 *
 * @param[in] FAST_WF_CONNECTED_AP_INFO_V2_S *fast_ap_info
 * @return  OPRT_OS_ADAPTER_OK: success  Other: fail
 */
int tuya_os_adapt_wifi_fast_station_connect_v2(const FAST_WF_CONNECTED_AP_INFO_V2_S *fast_ap_info)
{
    return OPRT_OS_ADAPTER_OK;
}

/**
 * @brief set wifi country code
 *
 * @param[in]       ccode  country code
 * @return  OPRT_OS_ADAPTER_OK: success  Other: fail
 */
int tuya_os_adapt_wifi_set_country_code(const COUNTRY_CODE_E ccode)
{
    int ret = OPRT_OS_ADAPTER_OK;
    int country_num = sizeof(country_code) / sizeof(wifi_country_t);
    wifi_country_t *country = (ccode < country_num) ? &country_code[ccode] : &country_code[COUNTRY_CODE_CN];

    ret = bk_wlan_set_country(country);
    if (ret != OPRT_OS_ADAPTER_OK) {
        LOG_ERR("set country err!\r\n");
    }

    if (ccode == COUNTRY_CODE_EU) { //enable
        bk_wlan_phy_open_cca();
    } else { //disable
        bk_wlan_phy_close_cca();
    }

    return OPRT_OS_ADAPTER_OK;
}

/**
 * @brief send wifi management
 *
 * @param[in]       buf         pointer to buffer
 * @param[in]       len         length of buffer
 * @return  OPRT_OS_ADAPTER_OK: success  Other: fail
 */
int tuya_os_adapt_wifi_send_mgnt(const unsigned char *buf, const unsigned int len)
{
    int ret = bk_wlan_send_80211_raw_frame((unsigned char *)buf, len);
    if (ret < 0) {
        return OPRT_OS_ADAPTER_MGNT_SEND_FAILED;
    }

    return OPRT_OS_ADAPTER_OK;
}

/**
 * @brief 进入低功耗处理
 *
 * @param[in] none
 * @return  none
 */
static void ty_wifi_powersave_enable(void)
{
    bk_wlan_stop_ez_of_sta();
    rtos_delay_milliseconds(50);
    bk_wlan_dtim_rf_ps_mode_enable();
    bk_wlan_dtim_rf_ps_timer_start();
    lp_status = 1;
}

/**
 * @brief 退出低功耗处理
 *
 * @param[in] none
 * @return  none
 */
static void ty_wifi_powersave_disable(void)
{
    bk_wlan_dtim_rf_ps_mode_disable();
    bk_wlan_dtim_rf_ps_timer_pause();
    rtos_delay_milliseconds(50);
    bk_wlan_start_ez_of_sta();
    lp_status = 0;
}

/**
 * @brief register receive wifi management callback
 *
 * @param[in]       enable
 * @param[in]       recv_cb     receive callback
 * @return  OPRT_OS_ADAPTER_OK: success  Other: fail
 */
int tuya_os_adapt_wifi_register_recv_mgnt_callback(const BOOL_T enable, const WIFI_REV_MGNT_CB recv_cb)
{
    LOG_DEBUG("tuya_os_adapt_wifi_register_recv_mgnt_callback enable %d\r\n",enable);
    if (enable) {
        WF_WK_MD_E mode;
        int ret = tuya_os_adapt_wifi_get_work_mode(&mode);
        if (OPRT_OS_ADAPTER_OK != ret) {
            LOG_ERR("set rece mgnt cb, get mode err\r\n");
            return OPRT_OS_ADAPTER_COM_ERROR;
        }

        if ((mode == WWM_LOWPOWER) || (mode == WWM_SNIFFER)) {
            LOG_ERR("set rece mgnt cb, but mode is err\r\n");
            return OPRT_OS_ADAPTER_COM_ERROR;
        }

        ty_wifi_powersave_disable();        /* 强制退出低功耗的模式 */
        mgnt_recv_cb = recv_cb;
        bk_wlan_reg_rx_mgmt_cb((mgmt_rx_cb_t)wifi_mgnt_frame_rx_notify, 2); /* 将回调注册到原厂中 */
        if (HW_IDLE == nxmac_current_state_getf()) {
            rw_msg_send_mm_active_req();//强制激活mac控制层
        }
    } else {
        bk_wlan_reg_rx_mgmt_cb(NULL, 2);
        mgnt_recv_cb = NULL;
    }
    return OPRT_OS_ADAPTER_OK;
}

/**
 * @brief set wifi lowerpower mode
 *
 * @param[in]       en
 * @param[in]       dtim
 * @return  OPRT_OS_ADAPTER_OK: success  Other: fail
 */
int tuya_os_adapt_set_wifi_lp_mode(const BOOL_T en, const unsigned char dtim)
{
    if (TRUE == en) {
        if (lp_status) {
            LOG_ERR("bk wlan ps mode has enable\r\n");
            return OPRT_OS_ADAPTER_COM_ERROR;
        }

        ty_wifi_powersave_enable();
        LOG_DEBUG("bk_wlan_ps_mode_enable\r\n");
    } else {

        ty_wifi_powersave_disable();
        bk_wlan_mcu_ps_mode_disable();
        LOG_DEBUG("bk_wlan_ps_mode_disable\r\n");
    }

    return OPRT_OS_ADAPTER_OK;
}

extern int manual_cal_rfcali_status(void);
/**
 * @brief get wifi rf param exist or not
 *
 * @param[in]       none
 * @return  true: rf param exist  Other: fail
 */
bool_t tuya_os_adapt_wifi_rf_calibrated(void)
{
    int stat = manual_cal_rfcali_status();

    if (stat) {
        return true;
    }

    return false;
}

OPERATE_RET tuya_os_adapt_reg_wifi_intf(void)
{
    return tuya_os_adapt_reg_intf(INTF_WIFI, (void *)&m_tuya_os_wifi_intfs);
}

