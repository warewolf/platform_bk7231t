 /*============================================================================
 *                                                                            *
 * Copyright (C) by Tuya Inc                                                  *
 * All rights reserved                                                        *
 *                                                                            *
 * @author  :   wuls                                                         *
 * @date    :   2019-10-10                                                    *
 * @brief   :                                                                 *
 *                                                                            *
 =============================================================================*/


/*============================ INCLUDES ======================================*/
#include "tuya_adc.h"
#include "typedef.h"
#include "saradc_pub.h"
#include "drv_model_pub.h"
#include "FreeRTOS.h"


/*============================ MACROS ========================================*/
#define ADC_DEV_NUM         1
#define ADC_BUF_SIZE        1

/*============================ TYPES =========================================*/



/*============================ PROTOTYPES ====================================*/
static int adc_dev_init    (tuya_adc_t *adc, tuya_adc_cfg_t *cfg);
static int adc_dev_convert    (tuya_adc_t *adc, uint16_t *result);
static int adc_dev_control(tuya_adc_t *adc, uint8_t cmd, VOID *arg);
static int adc_dev_deinit(tuya_adc_t *adc);

/*============================ LOCAL VARIABLES ===============================*/
static unsigned char g_adc_init = FALSE;
static unsigned short adc_buf[ADC_BUF_SIZE];

static tuya_adc_t s_adc_dev[ADC_DEV_NUM];
static saradc_desc_t adc_desc;  //ADC结构体

static const tuya_adc_ops_t  adc_dev_ops = {
    .init       = adc_dev_init,
    .convert    = adc_dev_convert,
    .control    = adc_dev_control,
    .deinit     = adc_dev_deinit,
};
/*============================ GLOBAL VARIABLES ==============================*/
/*============================ IMPLEMENTATION ================================*/
int platform_adc_init(void)
{
    int  i = 0;

    for (i = 0; i < ADC_DEV_NUM; i++) {
        s_adc_dev[i].ops = &adc_dev_ops;
        tuya_driver_register(&s_adc_dev[i].node, TUYA_DRV_ADC, i);
    }
    
    return OPRT_OK;
}

static int adc_dev_init    (tuya_adc_t *adc, tuya_adc_cfg_t *cfg)
{
    adc_desc.channel = 3;
    adc_desc.current_read_data_cnt = 0;
    adc_desc.current_sample_data_cnt = 0;
    adc_desc.pData = &adc_buf[0];
    adc_desc.data_buff_size = ADC_BUF_SIZE;
    adc_desc.mode = 3;//adc->mode;  //后续可以更改！在cfg中进行配置
    adc_desc.filter = 0;
    adc_desc.has_data = 0;
    adc_desc.all_done = 0;
    adc_desc.pre_div = 8;
    adc_desc.samp_rate = 2; //后续可以更改！在cfg中进行配置
    adc_desc.p_Int_Handler = saradc_disable;

    g_adc_init = TRUE;
}

/**
 * @brief: ADC 适配转换，ADC检测范围0~2.4V 0~4096
 * @param[in]: 
 * @return: OPRT_OK
 */
static int adc_dev_convert    (tuya_adc_t *adc, uint16_t *result)
{
    unsigned char i = 0;
    unsigned int status;
    int adc_hdl;
    static unsigned short last_adc = 0;

    if(!g_adc_init){
        bk_printf("adc not init!\r\n");
        return OPRT_COM_ERROR;
    }
    
    GLOBAL_INT_DECLARATION();

    *(unsigned short*)result = 0xFFFF;
    
    memset(adc_desc.pData, 0, adc_desc.data_buff_size * SIZEOF(unsigned short));

    GLOBAL_INT_DISABLE();

    adc_hdl = ddev_open(SARADC_DEV_NAME, &status, (unsigned int)&adc_desc); 
    if ((DD_HANDLE_UNVALID == adc_hdl) || (SARADC_SUCCESS != status))
    {
        if (SARADC_SUCCESS != status)
        {
            ddev_close(adc_hdl);
        }
        adc_hdl = DD_HANDLE_UNVALID;
        GLOBAL_INT_RESTORE();
        bk_printf("tuya_hal_adc_value_get ddev_open error:%d\r\n", status);
        return -1;
    }
    GLOBAL_INT_RESTORE();

    while (adc_desc.all_done == 0)
    {
        i ++;
        if (i > 100)
        {
            bk_printf("tuya_hal_adc_value_get: timeout!\r\n");
            break;
        }
        vTaskDelay(1);
    }
    
    if(adc_desc.current_sample_data_cnt >= 1) //此处简化直接就是采集1次，1这个数值后续可以优化
    {
        *(USHORT_T*)result = adc_desc.pData[0];
        adc_desc.has_data = 0;
        adc_desc.current_sample_data_cnt = 0;
        last_adc = *(USHORT_T*)result;
    } else {
        *(USHORT_T*)result = last_adc;
    }
    
    ddev_close(adc_hdl);

    return OPRT_OK;
}


static int adc_dev_control(tuya_adc_t *adc, uint8_t cmd, VOID *arg)
{
    unsigned short adc_buf = 0;
    int ret;
    
    switch(cmd) {
        case TUYA_ADC_VOLTAGE_CMD:

            ret = adc_dev_convert(adc, &adc_buf);
            if(ret != OPRT_OK) {
                return OPRT_COM_ERROR;
            }
            *(unsigned int *)arg = (adc_buf / 4096.0 * 2400); // 单位mv
            
            break;
       default:
            break;
    }
    return OPRT_OK;
}

static int adc_dev_deinit(tuya_adc_t  *asc)
{
    return OPRT_OK;
}
