#ifndef __CAN_POTOCOL_H
#define __CAN_POTOCOL_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"

/* Exported macro ------------------------------------------------------------*/
#define GIMBAL_CAN_ID_YAW	0x205U
#define GIMBAL_CAN_ID_PIT	0x206U  //0x20AU
#define TURNPLATE_CAN_ID	0x207U

/* Exported types ------------------------------------------------------------*/
typedef enum {
    CAN_STDID_0x1FF = 0,
    CAN_STDID_0x200 = 1,
    CAN_STDID_0x2FF = 2,
    CAN_STDID_COUNT = 3,
}can_stdid_list_t;

/* Exported functions --------------------------------------------------------*/
void	CAN_SendSingleData(drv_can_t *drv, int16_t txData);
void 	CAN_SendDataBuff(drv_type_t drv_type, uint32_t std_id, int16_t *txBuff);
void    CAN_AddTxMessage(drv_can_t *drv, int16_t txData);
void    CAN_StartTx(drv_can_t *drv);

#endif
