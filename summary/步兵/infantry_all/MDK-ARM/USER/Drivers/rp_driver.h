#ifndef __RP_DRIVER_H
#define __RP_DRIVER_H

#include "stm32f4xx_hal.h"





/* 驱动层 --------------------------------------------------------------------*/
/**
 *	@brief	驱动类型
 *	@class	driver
 */
typedef enum drv_type {
    DRV_CAN1,
    DRV_CAN2,
    DRV_PWM,
    DRV_IIC,
    DRV_UART1,
    DRV_UART2,
    DRV_UART3,
    DRV_UART4,
    DRV_UART5,
} drv_type_t;


/**
 *	@brief	iic驱动
 *	@class	driver
 */
typedef struct drv_iic {
    enum drv_type 	type;
} drv_iic_t;

/**
 *	@brief	can驱动
 *	@class	driver
 */
typedef struct drv_can {
    enum drv_type 	type;
    uint32_t		    can_id;
    uint32_t		    std_id;
    uint8_t			    drv_id;
    void			     (*tx_data)(struct drv_can *self, int16_t txData);
} drv_can_t;

/**
 *	@brief	pwm驱动
 *	@class	driver
 */
typedef struct drv_pwm {
    enum drv_type	type;
    void			(*output)(struct drv_pwm *self, int16_t pwm);
} drv_pwm_t;

/**
 *	@brief	uart驱动
 *	@class	driver
 */
typedef struct drv_uart {
    enum drv_type	type;
    void			(*tx_byte)(struct drv_uart *self, uint8_t byte);
} drv_uart_t;

#endif
