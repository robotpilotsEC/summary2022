#ifndef __IO_POTOCOL_H
#define __IO_POTOCOL_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"

/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
GPIO_PinState IO_ReadPin(drv_io_t *io);
void IO_WritePin(drv_io_t *io, GPIO_PinState io_state);

#endif
