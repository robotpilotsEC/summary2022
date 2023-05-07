#ifndef __SUPER_H
#define __SUPER_H
#include "rp_config.h"
#include "rp_math.h"
#include "S_function.h"
#include "DRIVE.h"
#include "cap_protocol.h"


/***************************************************************************/

typedef struct CAP_RP_struct {
 cap_send_data_t      *TX;
 cap_receive_data_t   *RX;
 uint16_t		       offline_cnt;
 uint16_t		       offline_max_cnt;	
 dev_work_state_t	 work_state; 


 void   (*ctrl)(void);	
 void		(*heart_beat)(void);	
 void		(*updata)(uint32_t canId, uint8_t *rxBuf);
} CAP_RP_t;




extern CAP_RP_t CAP_RP_2022;










#endif






