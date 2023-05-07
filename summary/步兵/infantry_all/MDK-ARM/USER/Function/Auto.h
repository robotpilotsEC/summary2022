#ifndef __AUTO_H
#define __AUTO_H

#include "rp_config.h"
#include "uart_drv.h"
#include  "vision_potocol.h"
#include  "vision_sensor.h"
#include "DEVICE.h"
#include "State.h"

#define H_LINE 1
#define L_LINE 0
#define Trigger_OK PDout(13) = 1 
#define Trigger_NO PDout(13) = 0 
#define Trigger_State PDout(13)


extern Vision_Cmd_Id_t AUTO_CMD_MODE;

void Vision_Get(void);
void Vision_TX(void);

void SHOT_AUTO_INIT(void);
bool SHOT_AUTO_JUDGE(void);
bool AUTO_SHOOT_CTRL(void);


state_t DF_STATE(void);
state_t ZM_STATE(void);
auto_ing_t AUTO_ING(void);

void Vision_IRQ(void);




#endif




