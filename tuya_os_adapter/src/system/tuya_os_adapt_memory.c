/**
 * @file tuya_os_adapt_memory.c
 * @brief �ڴ�����ӿ�
 * 
 * @copyright Copyright(C),2018-2020, Ϳѻ�Ƽ� www.tuya.com
 * 
 */
 
#include <FreeRTOS.h>
#include "tuya_os_adapt_memory.h"


/***********************************************************
*************************micro define***********************
***********************************************************/

/***********************************************************
*************************variable define********************
***********************************************************/
static const TUYA_OS_MEMORY_INTF m_tuya_os_memory_intfs = {
    .malloc  = tuya_os_adapt_system_malloc, 
    .free    = tuya_os_adapt_system_free,
};

static TUYA_MALLOC_FUNC_T s_internal_malloc_func = NULL;
static TUYA_FREE_FUNC_T   s_internal_free_func   = NULL;

/***********************************************************
*************************function define********************
***********************************************************/
/**
 * @brief tuya_os_adapt_system_malloc���ڷ����ڴ�
 * 
 * @param[in]       size        ��Ҫ������ڴ��С
 * @return  ����õ����ڴ�ָ��
 */


#if HAL_MEM_DEBUG
static int	start_record_os_adapt_mem = 0;

void tuya_os_adapt_system_mem_start(void)
{
	start_record_os_adapt_mem = 1;
}
void tuya_os_adapt_system_mem_stop(void)
{
	start_record_os_adapt_mem = 0;
}
int tuya_os_adapt_system_mem_get(void)
{
	return start_record_os_adapt_mem;
}
static int malloc_cnt = 0;
void *__tuya_os_adapt_system_malloc(size_t size,char *file,int line)
#else
void *tuya_os_adapt_system_malloc(const size_t size)
#endif
{
    void *pMalloc;

    pMalloc = pvPortMalloc(size);
#if HAL_MEM_DEBUG
    if(start_record_os_adapt_mem) {
        malloc_cnt++;
        if (file) {
            if(strstr(file,"print_") || strstr(file,"parse_") || \
               strstr(file,"cJSON_")) {
                return pMalloc;
            }
        }

    bk_printf("%s:%d cnt:%d malloc mp:%p reqSize:%d \r\n",file?file:"UNKNOWN",line,malloc_cnt,pMalloc,size);
    }
#endif
    if(pMalloc == NULL) {
        bk_printf("malloc fail, heap left size %d\r\n", tuya_os_adapt_system_getheapsize());
    }
    return pMalloc;
}

/**
 * @brief tuya_os_adapt_system_free�����ͷ��ڴ�
 * 
 * @param[in]       ptr         ��Ҫ�ͷŵ��ڴ�ָ��
 */
#if HAL_MEM_DEBUG
static int free_cnt = 0;
void __tuya_os_adapt_system_free(void      * ptr,char *file,int line)
#else
void tuya_os_adapt_system_free(void* ptr)
#endif
{
    if(ptr == NULL) {
        return;
    }

	vPortFree(ptr);

#if HAL_MEM_DEBUG
    if(start_record_os_adapt_mem) {
        free_cnt++;

        // delete cjson print info
        if (file) {
            if(strstr(file,"print_") || strstr(file,"parse_") || \
               strstr(file,"cJSON_")) {
                return;
            }
        }       

        bk_printf("%s:%d sub_cnt:%d free mp:%p\r\n",file?file:"UNKNOWN",line,(malloc_cnt-free_cnt),ptr);
    }
#endif
}
/* add begin: by sunkz, interface regist */
OPERATE_RET tuya_os_adapt_reg_memory_intf(void)
{
    return tuya_os_adapt_reg_intf(INTF_MEMORY, &m_tuya_os_memory_intfs);
}
/* add end */


int tuya_hal_set_mem_func(TUYA_MALLOC_FUNC_T malloc_func, TUYA_FREE_FUNC_T free_func)
{
    s_internal_malloc_func = malloc_func;
    s_internal_free_func = free_func;
    return 0;
}

void* tuya_hal_internal_malloc(const size_t size)
{
    if (s_internal_malloc_func) {
        return s_internal_malloc_func(size);
    } else {
        return pvPortMalloc(size);
    }
}

void tuya_hal_internal_free(void* ptr)
{
    if (s_internal_free_func) {
        s_internal_free_func(ptr);
    } else {
        vPortFree(ptr);
    }
}

