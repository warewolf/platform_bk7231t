/***********************************************************
*  File: uni_storge.c 
*  Author: nzy
*  Date: 20170920
***********************************************************/
#define __UNI_STORGE_GLOBALS
#include "drv_model_pub.h"
#include "flash_pub.h"

#include "tuya_os_adapt_storge.h"
// #include "../errors_compat.h"
// #include "tuya_os_adapt_storge.h"
#include "tuya_os_adapter_error_code.h"

/***********************************************************
*************************micro define***********************
***********************************************************/
#define PARTITION_SIZE (1 << 12) /* 4KB */
#define FLH_BLOCK_SZ PARTITION_SIZE

// flash map 
#define SIMPLE_FLASH_START (0x200000 - 0x3000 - 0xE000)
#define SIMPLE_FLASH_SIZE 0xE000 // 56k

#define SIMPLE_FLASH_SWAP_START (0x200000 - 0x3000)
#define SIMPLE_FLASH_SWAP_SIZE 0x3000 // 12k

#define SIMPLE_FLASH_KEY_ADDR  (0x200000 - 0x3000 - 0xE000 - 0x1000)            //4k

#define UF_PARTITION_START1     (0x200000 - 0x3000 - 0xE000 - 0x1000) - 0xB000
#define UF_PARTITION_SIZE1      0xB000                                          //44k

#define UF_PARTITION_START2     (0x1E0000 - 0x8000)
#define UF_PARTITION_SIZE2      0x8000                                          //32k

#define UF_PARTITION_START3     (0x132000 - 0x8000)
#define UF_PARTITION_SIZE3      0x8000                                          //32k

/***********************************************************
*************************variable define********************
***********************************************************/

/* add begin: by sunkz, interface regist */
static const TUYA_OS_STORAGE_INTF m_tuya_os_storage_intfs = {
    .read  = tuya_os_adapt_flash_read, 
    .write = tuya_os_adapt_flash_write,
    .erase = tuya_os_adapt_flash_erase,
    .get_storage_desc = tuya_os_adapt_storage_get_desc,
    .get_uf_desc = tuya_os_adapt_uf_get_desc
};
/* add end */

static UNI_STORAGE_DESC_S storage = {
    SIMPLE_FLASH_START,
    SIMPLE_FLASH_SIZE,
    FLH_BLOCK_SZ,
    SIMPLE_FLASH_SWAP_START,
    SIMPLE_FLASH_SWAP_SIZE,
    SIMPLE_FLASH_KEY_ADDR
};

static UF_PARTITION_TABLE_S uf_file = {
    .sector_size = PARTITION_SIZE,
    .uf_partition_num = 3,
    .uf_partition = {
        {UF_PARTITION_START1, UF_PARTITION_SIZE1},                        //44K
        {UF_PARTITION_START2, UF_PARTITION_SIZE2},                        //32K
        {UF_PARTITION_START3, UF_PARTITION_SIZE3},                        //32K
    }
};


/***********************************************************
*************************function define********************
***********************************************************/
/***********************************************************
*  Function: tuya_os_adapt_flash_read
*  Input: addr size
*  Output: dst
*  Return: none
***********************************************************/
int tuya_os_adapt_flash_read(const unsigned int addr, unsigned char *dst, const unsigned int size)
{
    unsigned int status;
    if(NULL == dst) {
        return OPRT_INVALID_PARM;
    }
    hal_flash_lock();

    DD_HANDLE flash_handle;
    flash_handle = ddev_open(FLASH_DEV_NAME, &status, 0);
    ddev_read(flash_handle, dst, size, addr);
    ddev_close(flash_handle);
    
    hal_flash_unlock();

    return OPRT_OS_ADAPTER_OK;
}

static unsigned int __uni_flash_is_protect_all(void)
{
    DD_HANDLE flash_handle;
    unsigned int status;
    unsigned int param;

    flash_handle = ddev_open(FLASH_DEV_NAME, &status, 0);
    ddev_control(flash_handle, CMD_FLASH_GET_PROTECT, (void *)&param);
    ddev_close(flash_handle);
    //PR_NOTICE("_uni_flash_is_protect_all:%x\r\n",param);
    return (FLASH_PROTECT_ALL == param);
}


/***********************************************************
*  Function: tuya_os_adapt_flash_write
*  Input: addr src size
*  Output: none
*  Return: none
***********************************************************/
int tuya_os_adapt_flash_write(const unsigned int addr, const unsigned char *src, const unsigned int size)
{
    DD_HANDLE flash_handle;
    unsigned int protect_flag;
    unsigned int status;
    unsigned int param;

    if(NULL == src) 
    {
        return OPRT_INVALID_PARM;
    }

    hal_flash_lock();

    protect_flag = __uni_flash_is_protect_all();
    flash_handle = ddev_open(FLASH_DEV_NAME, &status, 0);

    if(protect_flag)
    {
        param = FLASH_PROTECT_HALF;
        ddev_control(flash_handle, CMD_FLASH_SET_PROTECT, (void *)&param);
    }
    
    ddev_write(flash_handle, (char *)src, size, addr);

    protect_flag = __uni_flash_is_protect_all();

    if(protect_flag)
    {
        param = FLASH_PROTECT_ALL;
        ddev_control(flash_handle, CMD_FLASH_SET_PROTECT, (void *)&param);
    }

    ddev_close(flash_handle);
    hal_flash_unlock();

    return OPRT_OS_ADAPTER_OK;
}

/***********************************************************
*  Function: tuya_os_adapt_flash_erase
*  Input: addr size
*  Output: 
*  Return: none
***********************************************************/
int tuya_os_adapt_flash_erase(const unsigned int addr, const unsigned int size)
{
    unsigned short start_sec = (addr/PARTITION_SIZE);
    unsigned short end_sec = ((addr+size-1)/PARTITION_SIZE);
    unsigned int status;
    unsigned int i = 0;
    unsigned int sector_addr;
    DD_HANDLE flash_handle;
    unsigned int  param;
    unsigned int protect_flag;

    hal_flash_lock();

    protect_flag = __uni_flash_is_protect_all();
    flash_handle = ddev_open(FLASH_DEV_NAME, &status, 0);

    if(protect_flag)
    {
        param = FLASH_PROTECT_HALF;
        ddev_control(flash_handle, CMD_FLASH_SET_PROTECT, (void *)&param);
    }

    for(i = start_sec;i <= end_sec;i++) {
        sector_addr = PARTITION_SIZE*i;
        ddev_control(flash_handle, CMD_FLASH_ERASE_SECTOR,(void*)(&sector_addr));
    }

    protect_flag = __uni_flash_is_protect_all();

    if(protect_flag)
    {
        param = FLASH_PROTECT_ALL;
        ddev_control(flash_handle, CMD_FLASH_SET_PROTECT, (void *)&param);
    }
    
    ddev_close(flash_handle);

    hal_flash_unlock();
     
    return OPRT_OS_ADAPTER_OK;
}

UF_PARTITION_TABLE_S *tuya_os_adapt_uf_get_desc(void)
{
    return &uf_file;
}

/***********************************************************
*  Function: uni_get_storge_desc
*  Input: none
*  Output: none
*  Return: UNI_STORGE_DESC_S
***********************************************************/
UNI_STORAGE_DESC_S *tuya_os_adapt_storage_get_desc(void)
{
    return &storage;
}

/***********************************************************
*  Function: tuya_os_adapt_flash_set_protect
*  Input: protect flag
*  Output: 
*  Return: none
***********************************************************/
int tuya_os_adapt_flash_set_protect(const bool enable)
{
    DD_HANDLE flash_handle;
    unsigned int  param;
    unsigned int status;

    flash_handle = ddev_open(FLASH_DEV_NAME, &status, 0);

    if(enable)
    {
        param = FLASH_PROTECT_ALL;
        ddev_control(flash_handle, CMD_FLASH_SET_PROTECT, (void *)&param);
    }
    else
    {
        param = FLASH_PROTECT_HALF;
        ddev_control(flash_handle, CMD_FLASH_SET_PROTECT, (void *)&param);
    }
    
    ddev_close(flash_handle);
    return OPRT_OS_ADAPTER_OK;
}

/* add begin: by sunkz, interface regist */
OPERATE_RET tuya_os_adapt_reg_storage_intf(void)
{
    return tuya_os_adapt_reg_intf(INTF_STORAGE, &m_tuya_os_storage_intfs);
}
/* add end */

