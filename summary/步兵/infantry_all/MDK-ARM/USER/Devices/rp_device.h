#ifndef __RP_DEVICE_H
#define __RP_DEVICE_H

#include "stm32f4xx_hal.h"




/* 设备层 --------------------------------------------------------------------*/
/**
 *	@brief	设备id列表 drv_id
 *	@class	device
 */
typedef enum {
		DEV_ID_NONE,
    DEV_ID_RC,   
    DEV_ID_IMU, 

		DEV_ID_JUDGE,
	  DEV_ID_VISION,
	
    DEV_ID_POWER_0,
    DEV_ID_POWER_1,
    DEV_ID_POWER_2,
    DEV_ID_POWER_3,
		
    DEV_ID_TURN_0,
    DEV_ID_TURN_1,
    DEV_ID_TURN_2,
    DEV_ID_TURN_3,		

    DEV_ID_FRIC_L,
    DEV_ID_FRIC_R,
    DEV_ID_BOX,
    DEV_ID_BARREL,
		
    DEV_ID_GIMB_Y,
    DEV_ID_GIMB_P,
		DEV_ID_CNT,
} dev_id_t;


/**
 *	@brief	设备工作状态
 *	@class	device
 */
typedef enum {
    DEV_ONLINE,
    DEV_OFFLINE,
} dev_work_state_t;

/**
 *	@brief	错误代码
 *	@class	device
 */
typedef enum {
    NONE_ERR,		// 正常(无错误)
		DEV_TYPE_ERR,
    DEV_ID_ERR,		// 设备ID错误	
    DEV_INIT_ERR,	// 设备初始化错误
    DEV_DATA_ERR,	// 设备数据错误
} dev_errno_t;



/**
 *	@brief	设备结构体定义模板
 *	@class	device
 */


typedef struct device {
    void							*info;		// 自定义具体设备信息结构体
    void							*driver;	// 自定义具体设备驱动结构体
    void							(*init)(struct device *self);	// 设备初始化函数
    void							(*update)(struct device *self, uint8_t *rxBuf);	// 设备数据更新函数
    void							(*check)(struct device *self);	// 设备数据检查函数
    void							(*heart_beat)(struct device *self);	// 设备心跳包
    dev_work_state_t	work_state;	// 设备工作状态
    dev_errno_t				errno;		// 可自定义具体设备错误代码
    dev_id_t					id;			// 设备id
} device_t;



#endif
