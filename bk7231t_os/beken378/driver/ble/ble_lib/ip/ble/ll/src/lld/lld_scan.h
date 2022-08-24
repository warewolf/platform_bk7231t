/**
 ****************************************************************************************
 *
 * @file lld_scan.h
 *
 * @brief Declaration of functions used for advertising test mode driver
 *
 * Copyright (C) RivieraWaves 2017-2019
 *
 ****************************************************************************************
 */

#ifndef LLD_ADV_TEST_H_
#define LLD_ADV_TEST_H_

/**
 ****************************************************************************************
 * @addtogroup LLDADVTEST
 * @ingroup LLD
 * @brief Advertising test mode driver
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"

#if (LLD_SCAN_MODE)
#include "ea.h"
#include "common_bt.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/// Unique tag that represents SACN Test mode driver data
#define BLE_SCAN_TEST_MODE_ELT    (0xDA5AA5AD)

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Start Scanning test mode
 *
 * @param[in] win_size  Maximum reception window size before changing channel - in slots
 * @param[in] msg_id    Message Identifier to trigger when advertising report is received
 * @param[in] dst_id    Task Identifier that will handle end message
 *
 * @return COMMON_ERROR_NO_ERROR if succeed else see error code.
 ****************************************************************************************
 */
uint8_t lld_adv_test_scan_start(uint16_t win_size, uint16_t msg_id, uint16_t dst_id);

/**
 ****************************************************************************************
 * @brief Stop Scanning test mode
 *
 * @param[in] msg_id    Message Identifier to trigger when scanning driver is stopped
 *
 * @return COMMON_ERROR_NO_ERROR if succeed else see error code.
 ****************************************************************************************
 */
uint8_t lld_adv_test_scan_stop(uint16_t msg_id);

/**
 ****************************************************************************************
 * @brief End of Advertising test mode event (driver internal use)
 *
 * @param[in] elt  Event arbiter information
 *
 ****************************************************************************************
 */
void lld_adv_test_end_cb(struct ea_elt_tag *elt);

/**
 ****************************************************************************************
 * @brief RX ISR Callback - triggered when an RX descriptor is used  (driver internal use)
 *
 * @param[in] elt  Event arbiter information
 *
 ****************************************************************************************
 */
void lld_adv_test_rx_isr_cb(struct ea_elt_tag *elt);

/**
 ****************************************************************************************
 * @brief Initialize advertising test mode driver
 *
 * @param[in] reset  True if reset False if boot init
 *
 ****************************************************************************************
 */
void lld_adv_test_init(bool reset);

#endif // (BLE_EMB_PRESENT && LLD_SCAN_MODE)

#endif // LLD_ADV_TEST_H_

/// @} LLDADVTEST
