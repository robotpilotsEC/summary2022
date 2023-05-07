#ifndef __CAN_POTOCAL_H
#define __CAN_POTOCAL_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"
#include "can_drv.h"
#include "DEVICE.h"


/* Exported functions --------------------------------------------------------*/
void	CAN_SendSingleData(drv_can_t *drv, int16_t txData);
void 	CAN_SendDataBuff(drv_type_t drv_type, uint32_t std_id, int16_t *txBuff);

#endif


