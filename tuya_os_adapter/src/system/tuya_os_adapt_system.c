/**
 * @file tuya_os_adapt_system.c
 * @brief system底层操作接口
 *
 * @copyright Copyright(C),2018-2020, 涂鸦科技 www.tuya.com
 *
 */
#define _UNI_SYSTEM_GLOBAL

#include "tuya_os_adapt_system.h"
#include "tuya_os_adapt_wifi.h"
#include "tuya_os_adapter_error_code.h"
#include "tuya_os_adapt_output.h"

#include "wlan_ui_pub.h"
#include "mem_pub.h"
#include "wdt_pub.h"
#include "drv_model_pub.h"

#include "start_type_pub.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/***********************************************************
*************************micro define***********************
***********************************************************/

/***********************************************************
*************************variable define********************
***********************************************************/
static const TUYA_OS_SYSTEM_INTF m_tuya_os_system_intfs = {
    .get_systemtickcount   = tuya_os_adapt_get_systemtickcount,
    .get_tickratems        = tuya_os_adapt_get_tickratems,
    .system_sleep          = tuya_os_adapt_system_sleep,
    .system_reset          = tuya_os_adapt_system_reset,
    .watchdog_init_start   = tuya_os_adapt_watchdog_init_start,
    .watchdog_refresh      = tuya_os_adapt_watchdog_refresh,
    .watchdog_stop         = tuya_os_adapt_watchdog_stop,
    .system_getheapsize    = tuya_os_adapt_system_getheapsize,
    .system_get_rst_info   = tuya_os_adapt_system_get_rst_info,
    .system_get_rst_ext_info = NULL,
    .get_random_data       = tuya_os_adapt_get_random_data,
    .set_cpu_lp_mode       = tuya_os_adapt_set_cpu_lp_mode,
};

/***********************************************************
*************************function define********************
***********************************************************/
/**
 * @brief tuya_os_adapt_get_systemtickcount用于获取系统运行ticket
 * @return SYS_TICK_T
 */
SYS_TICK_T tuya_os_adapt_get_systemtickcount(void)
{
    return (SYS_TICK_T)xTaskGetTickCount();
}

/**
 * @brief tuya_os_adapt_get_tickratems用于获取系统ticket是多少个ms
 *
 * @return the time is ms of a system ticket
 */
unsigned int tuya_os_adapt_get_tickratems(void)
{
    return (unsigned int)portTICK_RATE_MS;
}

/**
 * @brief tuya_os_adapt_system_sleep用于系统sleep
 *
 * @param[in] msTime sleep time is ms
 */
void tuya_os_adapt_system_sleep(const unsigned long msTime)
{
    vTaskDelay((msTime) / (portTICK_RATE_MS));
}

/**
 * @brief tuya_os_adapt_system_isrstatus
 *
 * @return true
 * @return false
 */
bool tuya_os_adapt_system_isrstatus(void)
{
    if (0 !=  bk_wlan_get_INT_status()) {
        return TRUE;
    }

    return FALSE;
}

/**
 * @brief tuya_os_adapt_system_reset用于重启系统
 *
 */
void tuya_os_adapt_system_reset(void)
{
    LOG_NOTICE("tuya_os_adapt_system_reset\r\n");
    bk_reboot();
}

/**
 * @brief 用于初始化并运行watchdog
 *
 * @param[in] timeval watch dog检测时间间隔：如果timeval大于看门狗的
 * 最大可设置时间，则使用平台可设置时间的最大值，并且返回该最大值
 * @return int [out] 实际设置的看门狗时间
 */
unsigned int tuya_os_adapt_watchdog_init_start(const unsigned int timeval)
{

    unsigned int ret;
    //init
    int cyc_cnt = timeval * 1000;

    if (cyc_cnt > 0xFFFF) { /* 60s */
        cyc_cnt = 0xFFFF;
    }

    //init wdt
    ret = sddev_control(WDT_DEV_NAME, WCMD_SET_PERIOD, &cyc_cnt);

    // start wdt timer
    ret |= sddev_control(WDT_DEV_NAME, WCMD_POWER_UP, NULL);

    if (ret != 0) {
        bk_printf("watch dog set error!\r\n");
    }

    return 15; /* 在OTA的过程中，可能由于XIP和优先级的问题，导致喂狗不及时会watchdog重启，直接返回成15s！！ */
}

/**
 * @brief 用于刷新watch dog
 *
 */
void tuya_os_adapt_watchdog_refresh(void)
{
    if (sddev_control(WDT_DEV_NAME, WCMD_RELOAD_PERIOD, NULL) != 0) {
        bk_printf("refresh watchdog err!\r\n");
    }
}

/**
 * @brief 用于停止watch dog
 *
 */
void tuya_os_adapt_watchdog_stop(void)
{
    sddev_control(WDT_DEV_NAME, WCMD_POWER_DOWN, NULL);
}

/**
 * @brief tuya_os_adapt_system_getheapsize用于获取堆大小/剩余内存大小
 *
 * @return int <0: don't support  >=0: current heap size/free memory
 */
int tuya_os_adapt_system_getheapsize(void)
{
    return (int)xPortGetFreeHeapSize();
}

/**
 * @brief tuya_os_adapt_system_getMiniheapsize用于获取最小的剩余内存大小
 *
 * @return int <0: don't support  >=0: mini heap size/free memory
 */
int tuya_os_adapt_system_getMiniheapsize(void)
{
    return (int)xPortGetMinimumEverFreeHeapSize();
}

/**
 * @brief tuya_os_adapt_system_get_rst_info用于获取硬件重启原因
 *
 * @return 硬件重启原因
 */
TY_RST_REASON_E tuya_os_adapt_system_get_rst_info(void)
{
    unsigned char value = bk_misc_get_start_type() & 0xFF;
    TY_RST_REASON_E bk_value;

    switch (value) {
        case RESET_SOURCE_POWERON:
            bk_value = TY_RST_POWER_OFF;
            break;

        case RESET_SOURCE_REBOOT:
            bk_value = TY_RST_SOFTWARE;
            break;

        case RESET_SOURCE_WATCHDOG:
            bk_value = TY_RST_HARDWARE_WATCHDOG;
            break;

        case RESET_SOURCE_CRASH_XAT0:
        case RESET_SOURCE_CRASH_UNDEFINED:
        case RESET_SOURCE_CRASH_PREFETCH_ABORT:
        case RESET_SOURCE_CRASH_DATA_ABORT:
        case RESET_SOURCE_CRASH_UNUSED:
        case RESET_SOURCE_CRASH_PER_XAT0:
            bk_value = TY_RST_FATAL_EXCEPTION;
            break;

        default:
            bk_value = TY_RST_POWER_OFF;
            break;

    }

    LOG_NOTICE("bk_rst:%d tuya_rst:%d\r\n", value, bk_value);

    return bk_value;
}

/**
 * @brief tuya_os_adapt_get_random_data用于获取指定条件下的随机数
 *
 * @param[in] range
 * @return 随机值
 */
int tuya_os_adapt_get_random_data(const unsigned int range)
{
    unsigned int trange = range;

    if (range == 0) {
        trange = 0xFF;
    }

    static char exec_flag = FALSE;

    if (!exec_flag) {
        exec_flag = TRUE;
    }

    return (rand() % trange);
}

/**
 * @brief tuya_os_adapt_set_cpu_lp_mode -- set CPU lowpower mode
 *
 * @param[in] en
 * @param[in] mode
 * @return int 0=true，!0=false
 */
unsigned char cpu_lp_flag = 0;

int tuya_os_adapt_wifi_get_lp_mode(void)
{
    return cpu_lp_flag;
}

/**
 * @brief tuya_os_adapt_set_cpu_lp_mode用于设置cpu的低功耗模式
 *
 * @param[in] en
 * @param[in] mode
 * @return int 0=成功，非0=失败
 */
int tuya_os_adapt_set_cpu_lp_mode(const bool_t en, const TY_CPU_SLEEP_MODE_E mode)
{
    LOG_DEBUG("*******************************tuya_os_adapt_set_cpu_lp_mode,en = %d, mode = %d\r\n", en, mode);

    if (mode == TY_CPU_SLEEP) {
        if (en) {
            if (cpu_lp_flag == 0) {
                cpu_lp_flag = 1;
                LOG_DEBUG("pmu_release_wakelock(PMU_OS)\r\n");
            }

            bk_wlan_mcu_ps_mode_enable();
            LOG_DEBUG("bk_wlan_mcu_ps_mode_enable()\r\n");
        } else {
            bk_wlan_mcu_ps_mode_disable();
            LOG_DEBUG("bk_wlan_mcu_ps_mode_disable()\r\n");
        }
    } else {
        return OPRT_OS_ADAPTER_CPU_LPMODE_SET_FAILED;
    }

    return OPRT_OS_ADAPTER_OK;
}

OPERATE_RET tuya_os_adapt_reg_system_intf(void)
{
    return tuya_os_adapt_reg_intf(INTF_SYSTEM, (void *)&m_tuya_os_system_intfs);
}

