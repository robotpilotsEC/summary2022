#ifndef __IO_SENSOR_H
#define __IO_SENSOR_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"

/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef struct {
	uint8_t state;
	void	(*on);
	void	(*off);
	void	(*toggle);
}io_sensor_t;

io_sensor_t led_red;
io_sensor_t led_green;
io_sensor_t led_blue;
io_sensor_t led_orange;
io_sensor_t laser;

/* Exported functions --------------------------------------------------------*/

#endif
