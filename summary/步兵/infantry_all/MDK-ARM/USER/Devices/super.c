#include "super.h"

void super_heart_beat(void);
void Super_Rx_Data(uint32_t canId, uint8_t *rxBuf);

/******************************************************************************/

extern cap_send_data_t    cap_send_data;
extern cap_receive_data_t cap_receive_data;
void cap_rp_ctrl(void);
void cap_rp_heart_beat(void);

CAP_RP_t CAP_RP_2022 = {

 .TX       = &cap_send_data,
 .RX       = &cap_receive_data,
 .work_state = DEV_OFFLINE,	
 .offline_max_cnt = 5000,
 .heart_beat = cap_rp_heart_beat,
 .updata     = CAN_rxDataHandler,
 .ctrl       = cap_rp_ctrl,
};



void cap_rp_ctrl(void)
{
	set_message();
	
	if(CARR_MODE == 0){
			
		can_send_0x2E(2);
		can_send_0x2F(2);	
		
	}	
  else if(CARR_MODE == 1 || CARR_MODE == 2 || CARR_MODE == 3 || CARR_MODE == 4){
		
		can_send_0x2E(1);
		can_send_0x2F(1);
		
	}

}

void cap_rp_heart_beat(void)
{

	CAP_RP_2022.offline_cnt++;
	if(CAP_RP_2022.offline_cnt > CAP_RP_2022.offline_max_cnt) 
	{
		CAP_RP_2022.offline_cnt = CAP_RP_2022.offline_max_cnt;
		CAP_RP_2022.work_state = DEV_OFFLINE;
	}
	else 
	{
		if(CAP_RP_2022.work_state == DEV_OFFLINE)
			 CAP_RP_2022.work_state =  DEV_ONLINE;
	}
}






