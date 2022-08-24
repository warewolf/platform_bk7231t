/**
 * @file tuya_os_adapt_memory.h
 * @brief 内存操作接口封装
 * 
 * @copyright Copyright(C),2018-2020, 涂鸦科技 www.tuya.com
 * 
 */
 
#ifndef __TUYA_OS_ADAPT_MEMORY_H__
#define __TUYA_OS_ADAPT_MEMORY_H__


#include <stddef.h>
#include "tuya_os_adapter.h"

#ifdef __cplusplus
extern "C" {
#endif

#define HAL_MEM_DEBUG 0

#if HAL_MEM_DEBUG
void tuya_os_adapt_system_mem_start(void);
void tuya_os_adapt_system_mem_stop(void);
int tuya_os_adapt_system_mem_get(void);
void *__tuya_os_adapt_system_malloc(size_t size,char *file,int line);
void __tuya_os_adapt_system_free(void      * ptr,char *file,int line);
#define tuya_os_adapt_system_malloc(x) __tuya_os_adapt_system_malloc(x,__FILE__,__LINE__)
#define tuya_os_adapt_system_free(x) __tuya_os_adapt_system_free(x,__FILE__,__LINE__)
#else
/**
 * @brief tuya_os_adapt_system_malloc用于分配内存
 * 
 * @param[in]       size        需要分配的内存大小
 * @return  分配得到的内存指针
 */
void *tuya_os_adapt_system_malloc(const size_t size);

/**
 * @brief tuya_os_adapt_system_free用于释放内存
 * 
 * @param[in]       ptr         需要释放的内存指针
 */
void tuya_os_adapt_system_free(void* ptr);
#endif
/* add begin: by sunkz, interface regist */
OPERATE_RET tuya_os_adapt_reg_memory_intf(void);
/* add end */


/**
 * @brief 内存分配操作函数指针类型
 * 
 */
typedef void* (*TUYA_MALLOC_FUNC_T)(const size_t size);

/**
 * @brief 内存释放操作函数指针类型
 * 
 */
typedef void (*TUYA_FREE_FUNC_T)(void* ptr);

/**
 * @brief 设置内存操作函数
 *      Base 层的内存池管理初始化完成之后，通过此接口将内存操作函数设置给 OS 层
 * 
 * @param[in]       malloc_func 内存分配操作函数指针
 * @param[in]       free_func   内存释放操作函数指针
 * @retval          0           成功
 * @retval          非0         失败
 */
int tuya_hal_set_mem_func(TUYA_MALLOC_FUNC_T malloc_func, TUYA_FREE_FUNC_T free_func);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif // __TUYA_OS_ADAPT_MEMORY_H__

