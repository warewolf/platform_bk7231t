/**
 * @file tuya_os_adapt_bt.h
 * @brief BLE操作接口
 * 
 * @copyright Copyright(C),2018-2020, 涂鸦科技 www.tuya.com
 * 
 */

/*
 *  Copyright (C) 2014-2019, Tuya Inc., All Rights Reserved
 *  SPDX-License-Identifier: Apache-2.0
 *
 *  Licensed under the Apache License, Version 2.0 (the "License"); you may
 *  not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 *  This file is part of tuya ble sdk 
 */


#ifndef TUYA_OS_ADAPT_BT_H__
#define TUYA_OS_ADAPT_BT_H__


#include <stdbool.h>
#include <stdint.h>
#include "tuya_cloud_types.h"
#include "tuya_ble_type.h"
#include "tuya_os_adapter.h"

#if defined(TY_BT_MOD) && TY_BT_MOD == 1

//hz-低功耗时，只支持蓝牙配网，不支持蓝牙控制，该宏的作用只是框住相关代码，便于管理
#define ONLY_SUPPORT_BT_CONFIG      (1)

/**
 * @brief tuya_os_adapt_bt_port_init
 * 
 * @param[in] p 
 * @return OPERATE_RET 
 */
OPERATE_RET tuya_os_adapt_bt_port_init(ty_bt_param_t *p);

/**
 * @brief tuya_os_adapt_bt_port_deinit
 * 
 * @return OPERATE_RET 
 */
OPERATE_RET tuya_os_adapt_bt_port_deinit(void);

/**
 * @brief tuya_os_adapt_bt_gap_disconnect
 * 
 * @return OPERATE_RET 
 */
OPERATE_RET tuya_os_adapt_bt_gap_disconnect(void);

/**
 * @brief tuya_os_adapt_bt_send
 * 
 * @param[in] data: send buffer
 * @param[in] len: send buffer length
 * @return OPERATE_RET 
 */
OPERATE_RET tuya_os_adapt_bt_send(BYTE_T *data, UINT8_T len);

/**
 * @brief tuya_os_adapt_bt_reset_adv
 * 
 * @param[out] adv 
 * @param[out] scan_resp 
 * @return OPERATE_RET 
 */
OPERATE_RET tuya_os_adapt_bt_reset_adv(tuya_ble_data_buf_t *adv, tuya_ble_data_buf_t *scan_resp);

/**
 * @brief tuya_os_adapt_bt_get_rssi
 * 
 * @param[out] rssi 
 * @return OPERATE_RET 
 */
OPERATE_RET tuya_os_adapt_bt_get_rssi(SCHAR_T *rssi);

/**
 * @brief tuya_os_adapt_bt_start_adv
 * 
 * @return OPERATE_RET 
 */
OPERATE_RET tuya_os_adapt_bt_start_adv(void);

/**
 * @brief tuya_os_adapt_bt_stop_adv
 * 
 * @return OPERATE_RET 
 */
OPERATE_RET tuya_os_adapt_bt_stop_adv(void);

/**
 * @brief 
 * 
 * @param ty_bt_scan_info_t 
 * @return OPERATE_RET 
 */
OPERATE_RET tuya_os_adapt_bt_assign_scan(INOUT ty_bt_scan_info_t *info);

//给sdk使用：
OPERATE_RET tuya_os_adapt_bt_scan_init(IN TY_BT_SCAN_ADV_CB scan_adv_cb);
OPERATE_RET tuya_os_adapt_bt_start_scan(VOID_T);
OPERATE_RET tuya_os_adapt_bt_stop_scan(VOID_T);

/* add begin: by sunkz, interface regist */
OPERATE_RET tuya_os_adapt_reg_bt_intf(void);
/* add end */

#endif
#endif


