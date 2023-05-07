/*
*		ң���Լ�����
*
*
*/


#include "rc.h"
#include "State.h"
#include "S_function.h"
/****************************ʵ�廯*******************************/
static void rc_sensor_check(rc_sensor_t *rc);
static void rc_sensor_heart_beat(rc_sensor_t *rc);

void Key_Channel_Update(void);
void Mouse_Updata(void);
bool RC_IsChannelReset(void);


/* ������Ϣ */
Keyboard_Info_t Keyboard;

/* ң����Ϣ */
rc_sensor_info_t rc_info = {
	.offline_max_cnt = 60,
};
/* ң������ */
drv_uart_t rc_sensor_driver = {
	.type    = DRV_UART2,
	.tx_byte = NULL,
};

rc_sensor_t rc = {
  .info       = &rc_info,
	.driver     = &rc_sensor_driver,
  .init       = rc_sensor_init,
  .update     = rc_sensor_update,
  .check      = rc_sensor_check,
	.heart_beat = rc_sensor_heart_beat,
	.work_state = DEV_OFFLINE,
	.id         = DEV_ID_RC,
};



/*


*  ң��������գ�ʧ��ʱʹ��


*/
void RC_ResetData(rc_sensor_t *rc)
{
	rc->info->ch0 = 0;           // ͨ��ֵǿ�����ó��м�ֵ(������ҡ�˵�״̬)
	rc->info->ch1 = 0;
	rc->info->ch2 = 0;
	rc->info->ch3 = 0;	
	rc->info->s1  = 4;      
	rc->info->s2  = 4;
	
	rc->info->mouse.x        = 0;// ���
	rc->info->mouse.y        = 0;
	rc->info->mouse.z        = 0;
	rc->info->mouse.press_l  = 0;
	rc->info->mouse.press_r  = 0;	
	rc->info->kb.key_code    = 0;// ����	
	rc->info->thumbwheel     = 0;// ����
	rc->info->TW             = 0;
	
	for(char i=0; i<KEY_CNT; i++)//��λ���м��̱�־λ
		rc->info->Key_Flg[i] = 0;	
	
//	State.init();
}








/*

	State
	״̬�л���ʼ��

*/
void Rc_Init(void)
{
	rc.info->ch0 = 0;           // ͨ��ֵǿ�����ó��м�ֵ(������ҡ�˵�״̬)
	rc.info->ch1 = 0;
	rc.info->ch2 = 0;
	rc.info->ch3 = 0;	
	rc.info->thumbwheel     = 0;// ����
	rc.info->TW             = 0;
	State.SW2_LOCK = 0;
	
	rc.info->mouse.x        = 0;// ���
	rc.info->mouse.y        = 0;
	rc.info->mouse.z        = 0;
	rc.info->mouse.press_l  = 0;
	rc.info->mouse.press_r  = 0;	
	rc.info->kb.key_code    = 0;// ����	
	rc.info->thumbwheel     = 0;// ����

	for(char i=0; i<KEY_CNT; i++)//��λ���м��̱�־λ
	rc.info->Key_Flg[i] = 0;	
	
	State.CH_LOCK = 0;
}

void Key_Init(void)
{
	rc.info->mouse.x        = 0;// ���
	rc.info->mouse.y        = 0;
	rc.info->mouse.z        = 0;
	rc.info->mouse.press_l  = 0;
	rc.info->mouse.press_r  = 0;	
	rc.info->kb.key_code    = 0;// ����	
	rc.info->thumbwheel     = 0;// ����
	rc.info->TW             = 0;
	
	for(char i=0; i<KEY_CNT; i++)//��λ���м��̱�־λ
	rc.info->Key_Flg[i] = 0;	
	
	State.CH_LOCK = 0;
}
/*״̬�л���ʼ��end*/



/* 
		ң�ع��в�ѯ������
		һ�㸴λʱʹ�� 

*/
bool RC_IsChannelReset(void)
{
	if((DeathZoom(rc_info.ch0, 0, 1) == 0) && 
		 (DeathZoom(rc_info.ch1, 0, 1) == 0) && 
		 (DeathZoom(rc_info.ch2, 0, 1) == 0) && 
		 (DeathZoom(rc_info.ch3, 0, 1) == 0)   )	
	{
		return true;
	}
	return false;		
}



void Init_Rc(void)
{
	rc.init(&rc);
}

bool rc_offline_check(void)
{
	rc.heart_beat(&rc);
	if(rc.work_state == DEV_OFFLINE)
		return 1;
	
	else
		return 0 ;
}







								/*����Ϊ�ṹ�庯�� begin*/


void rc_sensor_init(rc_sensor_t *rc_sensor)
{
	rc_sensor->info->offline_cnt = rc_sensor->info->offline_max_cnt + 1;
	rc_sensor->work_state = DEV_OFFLINE;
	RC_ResetData(rc_sensor);
	if(rc_sensor->id == DEV_ID_RC)
		rc_sensor->errno = NONE_ERR;
	else
		rc_sensor->errno = DEV_ID_ERR;
}


  /*�������� �ڽ����ж���ʹ��*/
void rc_sensor_update(rc_sensor_t *rc_sensor , uint8_t *rxBuf)
{
	rc_sensor->info->ch0 = (rxBuf[0] | rxBuf[1] << 8) & 0x07FF;//0x07FF�պ�11λ����λ�󣬽�11λ�������ȥ
	rc_sensor->info->ch1 = (rxBuf[1]>>3 | rxBuf[2] << 5) & 0x07FF;
	rc_sensor->info->ch2 = (rxBuf[2]>>6 | rxBuf[3] << 2 | rxBuf[4] << 10) & 0x07FF;
	rc_sensor->info->ch3 = (rxBuf[4]>>1 | rxBuf[5] << 7) & 0x07FF;
	rc_sensor->info->s2  = (rxBuf[5]>>4) & 0x0003;
	rc_sensor->info->s1  = (rxBuf[5]>>6) & 0x0003;
	
	rc_sensor->info->ch0 -= 1024;
	rc_sensor->info->ch1 -= 1024;
	rc_sensor->info->ch2 -= 1024;
	rc_sensor->info->ch3 -= 1024;
		
	rc_sensor->info->mouse.x = rxBuf[6]  | (rxBuf[7 ] << 8);
	rc_sensor->info->mouse.y = rxBuf[8]  | (rxBuf[9 ] << 8);
	rc_sensor->info->mouse.z = rxBuf[10] | (rxBuf[11] << 8);
	rc_sensor->info->mouse.press_l = rxBuf[12];
	rc_sensor->info->mouse.press_r = rxBuf[13];
	
	rc_sensor->info->kb.key_code= rxBuf[14] | (rxBuf[15] << 8);	
	
	rc_sensor->info->thumbwheel  = ((int16_t)rxBuf[16] | ((int16_t)rxBuf[17] << 8)) & 0x07ff;
	rc_sensor->info->thumbwheel -= 1024;	

	if(rc_sensor->info->thumbwheel > 600)rc_sensor->info->TW  = 5;
	if(rc_sensor->info->thumbwheel <-600)rc_sensor->info->TW  = 0;

	Mouse_Updata();
	
	//����ͨ���󴥵��·賵
	if(RC_IsChannelReset())State.CH_LOCK = 1;
	
	if(!State.CH_LOCK){
		
		rc.info->ch0 = 0; // ͨ��ֵǿ�����ó��м�ֵ(������ҡ�˵�״̬)
		rc.info->ch1 = 0;
		rc.info->ch2 = 0;
		rc.info->ch3 = 0;	
		
	}
	
	rc_sensor->info->offline_cnt = 0;
}


	/*������� �ڽ����ж���ʹ��*/
static void rc_sensor_check(rc_sensor_t *rc)
{
	rc_sensor_info_t *rc_info = rc->info;
	
	if(abs(rc_info->ch0) > 660 ||
	   abs(rc_info->ch1) > 660 ||
	   abs(rc_info->ch2) > 660 ||
	   abs(rc_info->ch3) > 660)
	{
		rc->errno = DEV_DATA_ERR;
		rc_info->ch0 = 0;
		rc_info->ch1 = 0;
		rc_info->ch2 = 0;
		rc_info->ch3 = 0;		
		rc_info->s1  = SW_MID;
		rc_info->s2  = SW_MID;
		rc_info->thumbwheel = 0;
	}
	else
	{
		rc->errno = NONE_ERR;
		
	}		
}
		
	/*������ ��ʧ����麯���е���*/
static void rc_sensor_heart_beat(rc_sensor_t *rc)
{
	rc_sensor_info_t *rc_info = rc->info;

	rc_info->offline_cnt++;
	if(rc_info->offline_cnt > rc_info->offline_max_cnt)//ÿ�εȴ�һ��ʱ����Զ�����
	{
		rc_info->offline_cnt = rc_info->offline_max_cnt;
		rc->work_state = DEV_OFFLINE;
	} 
	else //ÿ�ν��ճɹ�����ռ���
	{
		/* ����->���� */
		if(rc->work_state == DEV_OFFLINE)
		{
			rc->work_state = DEV_ONLINE;
			RC_ResetData(rc);  //�������Ӻ�λ����ң������
		}
	}
}	
						/*����Ϊ�ṹ�庯�� end*/

/*
	���¼������

*/
void Rc_Data(uint8_t *rxBuf)
{
	rc.update(&rc, rxBuf);
	rc.check(&rc);
}



					   	/*���̲���*/

/*
*	���¼��
*	���²���
*/
void FirstGetInto_KEY_PRESS(KEY_Info_t *str)
{
  if(str->prev_KEY_PRESS != str->KEY_PRESS)
  {
    str->state_cnt = 0;
    str->prev_KEY_PRESS = str->KEY_PRESS;
  }
}

void KEY_State_Judge(KEY_Info_t *str , uint8_t KEY_PRESS , int change_tim ,int long_change_tim)
{
  str->KEY_PRESS = KEY_PRESS;
  FirstGetInto_KEY_PRESS(str);
  switch(KEY_PRESS)
  {
    case KEY_UP:  {
      if(str->prev_State != UP) 
      {
        str->state_cnt++;
        if(str->state_cnt >= change_tim)  
        {
          str->State = RELAX;
          str->prev_State = RELAX;
          if(str->state_cnt >= change_tim + 1)  //̧�𲻷ֳ���̧
          {
            str->State = UP;
            str->prev_State = UP;
          }
        }
      }else{str->state_cnt = 0;}
    }break;
    
    case KEY_DOWN:    {
      if(str->prev_State != DOWN) 
      {
        str->state_cnt++;
        if(str->state_cnt >= change_tim)  
        {
          str->State = PRESS;
          str->prev_State = PRESS;
          if(str->state_cnt >= change_tim + 1)
          {
            str->State = SHORT_DOWN;
            str->prev_State = SHORT_DOWN;
            if(str->state_cnt >= long_change_tim)  
            {
              str->State = DOWN;
              str->prev_State = DOWN;
            }
          }
        }
      }else{str->state_cnt = 0;}
    }break;
  }
}


						/*����ֵתΪң��ͨ��ֵ*/
	

/**
 * @brief ��꿨����
 * @note  �Ѽӻ����˲�
 * @param 
 */

float Mouse_X_UPDATA, Mouse_Y_UPDATA, Mouse_Z_UPDATA;
float Mouse_X_Last, Mouse_Y_Last, Mouse_Z_Last;
//6.46

float Mouse_X_Speed(void)
{
  float res;
	
	Mouse_X_Last = rc.info->Ch[0];
	
  if(abs(MOUSE_X_MOVE_SPEED > Xmax))res = 0;
  else 
		res = SF(MOUSE_X_MOVE_SPEED,Keyboard.MouseSF.SFX,0);
	
	rc.info->Ch[0] = (0.7f * MOUSE_X_RATE * res + 0.3f * Mouse_X_Last);
	
	if(rc.info->Ch[0]> 660)rc.info->Ch[0]= 660;
	if(rc.info->Ch[0]<-660)rc.info->Ch[0]=-660;
	
  return res;
}


float Mouse_Y_Speed(void)
{
  float res;
	
	Mouse_Y_Last = rc.info->Ch[1];
	
  if(abs(MOUSE_Y_MOVE_SPEED > Ymax))res = 0;
  else
		res = -SF(MOUSE_Y_MOVE_SPEED,Keyboard.MouseSF.SFY,0);

	rc.info->Ch[1] = (0.7f * MOUSE_Y_RATE * res + 0.3f * Mouse_Y_Last);
	
	if(rc.info->Ch[1]> 660)rc.info->Ch[1]= 660;
	if(rc.info->Ch[1]<-660)rc.info->Ch[1]=-660;
	
	if(abs(rc.info->Ch[1]) < 1)rc.info->Ch[1] = 0;
	
  return res;
}


float Mouse_Z_Speed(void)
{
  float res;
	
	Mouse_Z_Last = Mouse_Z_UPDATA;
	 
	res = SF(MOUSE_Z_MOVE_SPEED,Keyboard.MouseSF.SFZ,0);
	
	res = (0.8f * res + 0.2f * Mouse_Z_Last);
	
	rc.info->Mouse_ROLL = res;
	
  return res;
}



void Mouse_Updata(void)
{
	Mouse_X_UPDATA = Mouse_X_Speed();
	Mouse_Y_UPDATA = Mouse_Y_Speed();
	Mouse_Z_UPDATA = Mouse_Z_Speed();
}

void Mouse_FS(void)
{
	Mouse_X_Last = rc.info->Ch[0];
	Mouse_Y_Last = rc.info->Ch[1];
	
	rc.info->Ch[0] = (0.7f * MOUSE_X_RATE * Mouse_X_UPDATA + 0.3f * Mouse_X_Last);
  rc.info->Ch[1] = (0.7f * MOUSE_Y_RATE * Mouse_Y_UPDATA + 0.3f * Mouse_Y_Last);
	
  rc.info->Mouse_ROLL = Mouse_Z_UPDATA;
	
	if(rc.info->Ch[0]> 660)rc.info->Ch[0]= 660;
	if(rc.info->Ch[0]<-660)rc.info->Ch[0]=-660;
	
	if(rc.info->Ch[1]> 660)rc.info->Ch[1]= 660;
	if(rc.info->Ch[1]<-660)rc.info->Ch[1]=-660;
	
	
}
/*=======================================================*/
/*����ֵתΪң��ͨ��ֵ begin*/

//wsad�м�ͨ��ֵ��������������rc��ģ��ͨ��ֵCh[]
float W_Chan,
      S_Chan,
      A_Chan,
      D_Chan;


void Sim_Channel(float *CH, char dir,float num)//1up 0down
{
	if( dir)*CH+=num;
	if(!dir)*CH-=num;
	
	if(*CH > CH_MAX)     *CH = CH_MAX;
	if(*CH < CH_OFFSET  )*CH = CH_OFFSET;
}

/////////////////////////////////////
void WS_Channel(void) 
{
	int16_t CH;
	CH = W_Chan - S_Chan;	
	if(abs(CH) > CH_MAX)
	{
		if(CH > 0)CH = CH_MAX;
		if(CH < 0)CH = CH_MIN;
	}
	rc.info->Ch[3] = CH;
}

void AD_Channel(void) 
{
	int16_t CH;
	CH = D_Chan - A_Chan;
	if(abs(CH) > CH_MAX)
	{
		if(CH > 0)CH = CH_MAX;
		if(CH < 0)CH = CH_MIN;
	}
	rc.info->Ch[2] = CH;
}



#if CARR_MODE == 0

/*-��������-*/
float CH_UP   = 1.f;
float CH_DOWN = 3.f;
void Key_Channel_Update(void)
{
	if(KEY_W)Sim_Channel(&W_Chan, 1, CH_UP);
	else     Sim_Channel(&W_Chan, 0, CH_DOWN);
	
	if(KEY_S)Sim_Channel(&S_Chan, 1, CH_UP);
	else		 Sim_Channel(&S_Chan, 0, CH_DOWN);

	if(KEY_A)Sim_Channel(&A_Chan, 1, CH_UP);
	else     Sim_Channel(&A_Chan, 0, CH_DOWN);

	if(KEY_D)Sim_Channel(&D_Chan, 1, CH_UP);
	else     Sim_Channel(&D_Chan, 0, CH_DOWN);
	
	WS_Channel();
	AD_Channel();
}
#endif


#if CARR_MODE == 1
/*-��������-*/
float CH_UP   = 1.5f;
float CH_SLOW = 0.75f;
float CH_DOWN = 2.f;
void Key_Channel_Update(void)
{
	if                          (KEY_W)Sim_Channel(&W_Chan, 1, CH_UP);
	else if((KEY_A || KEY_D) && W_Chan)Sim_Channel(&W_Chan, 0, CH_SLOW);
	else                               Sim_Channel(&W_Chan, 0, CH_DOWN);
	
	if                          (KEY_S)Sim_Channel(&S_Chan, 1, CH_UP);
	else if((KEY_A || KEY_D) && S_Chan)Sim_Channel(&S_Chan, 0, CH_SLOW);
	else                               Sim_Channel(&S_Chan, 0, CH_DOWN);

	if                          (KEY_A)Sim_Channel(&A_Chan, 1, CH_UP);
	else if((KEY_W || KEY_S) && A_Chan)Sim_Channel(&A_Chan, 0, CH_SLOW);
	else                               Sim_Channel(&A_Chan, 0, CH_DOWN);

	if                          (KEY_D)Sim_Channel(&D_Chan, 1, CH_UP);
	else if((KEY_W || KEY_S) && D_Chan)Sim_Channel(&D_Chan, 0, CH_SLOW);
	else                               Sim_Channel(&D_Chan, 0, CH_DOWN);
	
	WS_Channel();
	AD_Channel();
}

#endif

#if CARR_MODE == 2
/*-��������-*/
float CH_UP   = 1.f;
float CH_DOWN = 3.f;
void Key_Channel_Update(void)
{
	if(KEY_W)Sim_Channel(&W_Chan, 1, CH_UP);
	else     Sim_Channel(&W_Chan, 0, CH_DOWN);
	
	if(KEY_S)Sim_Channel(&S_Chan, 1, CH_UP);
	else		 Sim_Channel(&S_Chan, 0, CH_DOWN);

	if(KEY_A)Sim_Channel(&A_Chan, 1, CH_UP);
	else     Sim_Channel(&A_Chan, 0, CH_DOWN);

	if(KEY_D)Sim_Channel(&D_Chan, 1, CH_UP);
	else     Sim_Channel(&D_Chan, 0, CH_DOWN);
	
	WS_Channel();
	AD_Channel();
}
#endif

#if CARR_MODE == 3
/*-��������-*/
float CH_UP   = 1.f;
float CH_DOWN = 3.f;
void Key_Channel_Update(void)
{
	if(KEY_W)Sim_Channel(&W_Chan, 1, CH_UP);
	else     Sim_Channel(&W_Chan, 0, CH_DOWN);
	
	if(KEY_S)Sim_Channel(&S_Chan, 1, CH_UP);
	else		 Sim_Channel(&S_Chan, 0, CH_DOWN);

	if(KEY_A)Sim_Channel(&A_Chan, 1, CH_UP);
	else     Sim_Channel(&A_Chan, 0, CH_DOWN);

	if(KEY_D)Sim_Channel(&D_Chan, 1, CH_UP);
	else     Sim_Channel(&D_Chan, 0, CH_DOWN);
	
	WS_Channel();
	AD_Channel();
}
#endif

#if CARR_MODE == 4
/*-��������-*/
float CH_UP   = 1.f;
float CH_DOWN = 3.f;
void Key_Channel_Update(void)
{
	if(KEY_W)Sim_Channel(&W_Chan, 1, CH_UP);
	else     Sim_Channel(&W_Chan, 0, CH_DOWN);
	
	if(KEY_S)Sim_Channel(&S_Chan, 1, CH_UP);
	else		 Sim_Channel(&S_Chan, 0, CH_DOWN);

	if(KEY_A)Sim_Channel(&A_Chan, 1, CH_UP);
	else     Sim_Channel(&A_Chan, 0, CH_DOWN);

	if(KEY_D)Sim_Channel(&D_Chan, 1, CH_UP);
	else     Sim_Channel(&D_Chan, 0, CH_DOWN);
	
	WS_Channel();
	AD_Channel();
}
#endif

#if CARR_MODE == 5
/*-��������-*/
float CH_UP   = 1.f;
float CH_DOWN = 3.f;
void Key_Channel_Update(void)
{
	if(KEY_W)Sim_Channel(&W_Chan, 1, CH_UP);
	else     Sim_Channel(&W_Chan, 0, CH_DOWN);
	
	if(KEY_S)Sim_Channel(&S_Chan, 1, CH_UP);
	else		 Sim_Channel(&S_Chan, 0, CH_DOWN);

	if(KEY_A)Sim_Channel(&A_Chan, 1, CH_UP);
	else     Sim_Channel(&A_Chan, 0, CH_DOWN);

	if(KEY_D)Sim_Channel(&D_Chan, 1, CH_UP);
	else     Sim_Channel(&D_Chan, 0, CH_DOWN);
	
	WS_Channel();
	AD_Channel();
}
#endif

/*����ֵתΪң��ͨ��ֵ end*/



/**********************************************/

