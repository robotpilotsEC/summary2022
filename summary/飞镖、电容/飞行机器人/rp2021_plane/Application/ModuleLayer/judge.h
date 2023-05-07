#ifndef __JUDGE_H
#define __JUDGE_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"

#include "judge_sensor.h"

/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef struct {
	judge_sensor_t		*dev;
	bool				(*if_data_valid)(void);
    color_t             (*my_color)(void);
	uint16_t			(*get_17mm_heat_limit)(void);
	uint16_t			(*get_17mm_heat_real)(void);
	uint8_t				(*get_17mm_speed_limit)(void);
} judge_t;

extern judge_t judge;
/* Exported functions --------------------------------------------------------*/

#endif
