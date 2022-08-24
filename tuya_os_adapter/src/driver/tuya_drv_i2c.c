 /*============================================================================
 *                                                                            *
 * Copyright (C) by Tuya Inc                                                  *
 * All rights reserved                                                        *
 *                                                                            *
 * @author  :   xbf                                                           *
 * @date    :   2019-08-27                                                    *
 * @brief   :                                                                 *
 *                                                                            *
 =============================================================================*/


/*============================ INCLUDES ======================================*/

#include "tuya_i2c.h"
#include "BkDriverGpio.h"
#include "gpio_pub.h"

/*============================ MACROS ========================================*/
#define I2C_SET_SDA(__LEVEL)    iic_sda_write(__LEVEL)
#define I2C_SET_SCL(__LEVEL)    iic_scl_write(__LEVEL)
#define I2C_GET_SDA()           iic_sda_read()
/*============================ MACROFIED FUNCTIONS ===========================*/
#define I2C_DELAY()             tuya_soft_i2c_delay(sw_i2c->cfg.delay_count)

#define I2C_SDA_H()             I2C_SET_SDA(1)
#define I2C_SDA_L()             I2C_SET_SDA(0)
#define I2C_SCL_L()             I2C_SET_SCL(0)
#define I2C_SCL_H()             I2C_SET_SCL(1)

#define SDA_PIN_INDEX           1      
#define SCL_PIN_INDEX           0

/*============================ TYPES =========================================*/
typedef enum {
    I2C_OK,
    I2C_NACK,
    I2C_ERR,
    I2C_TIMEOUT,
} i2c_result_t;

typedef struct {
    tuya_i2c_t          i2c;
    tuya_i2c_sw_cfg_t   cfg;
} tuya_sw_i2c_t;

/*============================ PROTOTYPES ====================================*/
static int sw_i2c_init      (tuya_i2c_t *i2c, tuya_i2c_cfg_t *cfg);
static int sw_i2c_xfer      (tuya_i2c_t *i2c, tuya_i2c_msg_t *msgs, uint8_t num);
static int sw_i2c_control   (tuya_i2c_t *i2c, uint8_t cmd, void *arg);
static int sw_i2c_deinit    (tuya_i2c_t *i2c);

/*============================ LOCAL VARIABLES ===============================*/
static const tuya_i2c_ops_t   sw_i2c_ops = {
    .init    = sw_i2c_init,
    .xfer    = sw_i2c_xfer,
    .control = sw_i2c_control,
    .deinit  = sw_i2c_deinit,
};

tuya_sw_i2c_t *sw_i2c;
static uint8_t sda_out=0;
/*============================ GLOBAL VARIABLES ==============================*/
/*============================ IMPLEMENTATION ================================*/
void iic_sda_write(int Level)
{   
    if(Level){
        BkGpioInitialize(SDA_PIN_INDEX, INPUT_PULL_UP);
    }else{
        BkGpioInitialize(SDA_PIN_INDEX, OUTPUT_NORMAL);
        BkGpioOutputLow(SDA_PIN_INDEX);
    }    
}

int iic_sda_read()
{   
    return BkGpioInputGet(SDA_PIN_INDEX);
}

void iic_scl_write(int Level)
{
   if(Level){
        BkGpioInitialize(SCL_PIN_INDEX, INPUT_PULL_UP);
    }else{
        BkGpioInitialize(SCL_PIN_INDEX, OUTPUT_NORMAL);
        BkGpioOutputLow(SCL_PIN_INDEX);
    }    
}


int iic_scl_read()
{

}

void tuya_soft_i2c_delay(uint32_t delay)
{
    unsigned char i = 5;

    while(i--);
}

static void i2c_start()
{   
    I2C_SDA_H();
    I2C_SCL_H();
    I2C_DELAY();

    if(!iic_sda_read()){
        bk_printf("ucSocI2CSdaInputRead error low!\r\n");
        return ;
    }

    I2C_SDA_L();
    I2C_DELAY();
    if(iic_sda_read()){
        bk_printf("ucSocI2CSdaInputRead error hight!\r\n");
        return ;
    }

    I2C_SDA_L();
    I2C_DELAY();
}

static void i2c_restart()
{
    I2C_SDA_H();
    I2C_SCL_H();
    I2C_DELAY();

    if(!iic_sda_read()){
        bk_printf("ucSocI2CSdaInputRead error low!\r\n");
        return ;
    }

    I2C_SDA_L();
    I2C_DELAY();
    if(iic_sda_read()){
        bk_printf("ucSocI2CSdaInputRead error hight!\r\n");
        return ;
    }

    I2C_SDA_L();
    I2C_DELAY();
}

static void i2c_stop()
{   
    I2C_SCL_L();
    I2C_DELAY();
    I2C_SDA_L();
    I2C_DELAY();
    I2C_SCL_H();
    I2C_DELAY();
    I2C_SDA_H();
    I2C_DELAY();
}

static i2c_result_t i2c_waitack()
{
    uint16_t time = 0;
    I2C_SCL_L();
    I2C_DELAY();
    I2C_SDA_H();
    I2C_DELAY();
    I2C_SCL_H();
    I2C_DELAY();
    while(I2C_GET_SDA()){
        time++;
        if(time > 240){
            I2C_SCL_L();
            return I2C_NACK;
        }
    }
    I2C_SCL_L();
    return I2C_OK;  
}

static i2c_result_t i2c_write_byte(uint8_t data, uint16_t flags)
{
    uint8_t cnt;

    for(cnt=0; cnt<8; cnt++){
        I2C_SCL_L();
        I2C_DELAY();
        if(data & 0x80){
            I2C_SDA_H();
        }
        else{
            I2C_SDA_L();
        }
        data <<= 1;
        I2C_DELAY();
        I2C_SCL_H();
        I2C_DELAY();
    }

    //! compatible  with non-standard i2c
    if (flags & TUYA_I2C_NO_WRITE_ACK) {
        return I2C_OK;
    }

    I2C_SCL_L();
    return i2c_waitack();
}

static i2c_result_t i2c_read_byte(uint8_t *byte)
{
    int i;
    uint8_t data = 0;
    I2C_SDA_H();
    for (i = 0; i < 8; i++) {
        I2C_SCL_L();
        I2C_DELAY();
        data <<= 1;
        I2C_SCL_H();
        if (I2C_GET_SDA()) {
            data |= 1;
        }
    }

    *byte = data;
    I2C_SCL_L();
    return I2C_OK;
}

static i2c_result_t i2c_send_data(tuya_sw_i2c_t *sw_i2c, tuya_i2c_msg_t *msg)
{
    i2c_result_t       result;

    while (msg->len > 0) {
        result = i2c_write_byte(*msg->buf++, msg->flags);
        if (I2C_OK == result) {
            msg->len --;
        } else if (I2C_NACK == result) {
            if (msg->flags & TUYA_I2C_IGNORE_NACK) {
                msg->len --;
                continue;
            } 
            bk_printf("i2c send bytes: NACK.\n");
            return I2C_ERR;
        } else {
            bk_printf("i2c send bytes: error %d\n", result);
            return I2C_ERR;
        }
    }

    return I2C_OK;
}


static i2c_result_t i2c_send_ack_or_nack(tuya_sw_i2c_t *sw_i2c, uint8_t ack)
{
    if (ack) {
        I2C_SET_SDA(0);
    }
    I2C_DELAY();
    I2C_SCL_H();
    I2C_SCL_L();

    return I2C_OK;
}

static i2c_result_t i2c_recv_bytes(tuya_sw_i2c_t *sw_i2c, tuya_i2c_msg_t *msg)
{
    i2c_result_t result;

    while (msg->len > 0) {
        result = i2c_read_byte(msg->buf++);
        if (I2C_OK != result) {
            break;
        }
        msg->len--;
        if (!(msg->flags & TUYA_I2C_NO_READ_ACK)) {
            result = i2c_send_ack_or_nack(sw_i2c, msg->len);
            if (I2C_OK != result) {
                return I2C_ERR;
            }
        }
    }

    return result;
}

static int i2c_send_address(tuya_sw_i2c_t *sw_i2c, 
                            uint8_t        addr,
                            uint8_t        retries,
                            uint16_t       flags)
{
    int i;
    i2c_result_t result;

    for (i = 0; i <= retries; i++) {
        result = i2c_write_byte(addr, flags);
        if (I2C_OK == result || i == retries) {
            break;
        }
        i2c_stop();
        I2C_DELAY();
        i2c_start();
    }

    return result;
}

static int sw_i2c_send_address(tuya_sw_i2c_t *sw_i2c, tuya_i2c_msg_t *msg)
{
    i2c_result_t   result;
    uint8_t addr1, addr2;
    uint8_t retries = 2;

    if (msg->flags & TUYA_I2C_ADDR_10BIT) {
        addr1 = 0xf0 | ((msg->addr >> 7) & 0x06);
        addr2 = msg->addr & 0xff;

        bk_printf("addr1: %d, addr2: %d\n", addr1, addr2);

        result = i2c_send_address(sw_i2c, addr1, retries, msg->flags);
        if ((I2C_OK != result) && !(msg->flags & TUYA_I2C_IGNORE_NACK)) {
            bk_printf("NACK: sending first addr\n");
            return I2C_ERR;
        }
        result = i2c_write_byte( addr2, msg->flags);
        if ((I2C_OK != result) && !(msg->flags & TUYA_I2C_IGNORE_NACK)) {
            bk_printf("NACK: sending second addr\n");
            return I2C_ERR;
        }
        if (msg->flags & TUYA_I2C_RD) {
            bk_printf("send repeated start condition\n");
            i2c_restart();
            addr1 |= 0x01;
            result = i2c_send_address(sw_i2c, addr1, retries, msg->flags);
            if ((I2C_OK != result) && !(msg->flags & TUYA_I2C_IGNORE_NACK)) {
                bk_printf("NACK: sending repeated addr\n");
                return I2C_ERR;
            }
        }
    } else {
        /* 7-bit addr */
        addr1 = msg->addr << 1;
        if (msg->flags & TUYA_I2C_RD) {
            addr1 |= 1;
        }
        result = i2c_send_address(sw_i2c, addr1, retries, msg->flags);
        if ((I2C_OK != result) && !(msg->flags & TUYA_I2C_IGNORE_NACK)) {
            bk_printf("NACK: sending repeated addr\r\n");
            return I2C_ERR;
        }
    }

    return I2C_OK;
}

static int sw_i2c_xfer(tuya_i2c_t *i2c, tuya_i2c_msg_t *msgs, uint8_t num)
{
    tuya_i2c_msg_t  *msg;
    i2c_result_t    rseult;
    BYTE_T i = 0;
    tuya_sw_i2c_t   *sw_i2c = (tuya_sw_i2c_t *)i2c;

    TUYA_DRV_ASSRET(NULL != sw_i2c);

    i2c_start();
    for (i = 0; i < num; i++) {
        msg = &msgs[i];
        if (!(msg->flags & TUYA_I2C_NO_START)) {
            if (i) {
                i2c_restart();
            }
            rseult = sw_i2c_send_address(sw_i2c, msg);
            if ((I2C_OK != rseult) && !(msg->flags & TUYA_I2C_IGNORE_NACK)) {
                bk_printf("receive NACK from device addr 0x%02x msg %d\r\n",
                        msgs[i].addr, i);
                goto __exit;
            }
        }
        if (msg->flags & TUYA_I2C_RD) {
            rseult = i2c_recv_bytes(sw_i2c, msg);
            if (rseult != I2C_OK) {
                goto __exit;
            }
        } else {
            rseult = i2c_send_data(sw_i2c, msg);
            if (rseult != I2C_OK) {
                goto __exit;
            }
        }
    }

__exit:
    i2c_stop(sw_i2c);
    return (I2C_OK == rseult) ? OPRT_OK : OPRT_COM_ERROR;
}

static int sw_i2c_init(tuya_i2c_t *i2c, tuya_i2c_cfg_t *i2c_cfg)
{
    tuya_sw_i2c_t   *sw_i2c = (tuya_sw_i2c_t *)i2c;

    TUYA_DRV_ASSRET(NULL != sw_i2c);

    BkGpioInitialize(SDA_PIN_INDEX, INPUT_PULL_UP);

    BkGpioInitialize(SCL_PIN_INDEX, INPUT_PULL_UP);

    return OPRT_OK;
}


static int sw_i2c_deinit(tuya_i2c_t *i2c)
{
    return OPRT_OK;
}

static int sw_i2c_control(tuya_i2c_t *i2c, uint8_t cmd, void *arg)
{
    return OPRT_OK;
}

int platform_iic_init()
{
    int result;
    
    sw_i2c = tuya_os_adapt_system_malloc(sizeof(tuya_sw_i2c_t));
    if (NULL == sw_i2c) {
        bk_printf("platform_iic_init fail ------\r\n");
        return OPRT_MALLOC_FAILED;
    }

    sw_i2c->i2c.ops = &sw_i2c_ops;
    sw_i2c->cfg.sda_pin = SDA_PIN_INDEX;
    sw_i2c->cfg.scl_pin = SCL_PIN_INDEX;
    sw_i2c->cfg.delay_count = 100;
    sw_i2c->cfg.timeout     = 100;

    return tuya_driver_register(&sw_i2c->i2c.node, TUYA_DRV_I2C, 0);
}
