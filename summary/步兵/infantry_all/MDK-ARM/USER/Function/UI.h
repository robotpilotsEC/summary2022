#ifndef UI_H
#define UI_H

#include "main.h"
#include "judge_infantrypotocol.h"

#define JUDGE_FRAME_HEADER 0xa5
/*����*/
#define Client_mid_position_x 960
#define Client_mid_position_y 540

/*׼������*/

#if 0 
//***********����ϵͳ�ķ�����Ϣ***************/
enum
{ 
	ID_game_state       						= 0x0001,//����״̬���ݣ�1Hz
	ID_game_result 	   							= 0x0002,//����������ݣ�������������
	ID_game_robot_HP       					= 0x0003,//����������Ѫ�����ݣ�1Hz����
	ID_dart_status									= 0x0004,//���ڷ���״̬�����ڷ���ʱ����
	ID_ICRA_buff_debuff_zone_status = 0x0005,//�˹�������ս���ӳ���ͷ���״̬��1Hz
	ID_event_data  									= 0x0101,//�����¼����ݣ��¼� �����ı�󡪡� ����
	ID_supply_projectile_action   	= 0x0102,//���ز���վ������ʶ���ݣ����� �����ı�󡪡� ����
	ID_supply_projectile_booking    = 0x0103,//���󲹸�վ�������� ���������ӷ��͡�����RM �Կ�����δ����10Hz
	ID_referee_warning					 		= 0x0104,//���о������ݣ���������󡪡�����
	ID_dart_remaining_time					= 0x0105,//���ڷ���ڵ���ʱ��1Hz
	ID_game_robot_state    					= 0x0201,//������״̬���ݣ�10Hz
	ID_power_heat_data    					= 0x0202,//ʵʱ�����������ݣ�50Hz
	ID_game_robot_pos        				= 0x0203,//������λ�����ݣ�10Hz
	ID_buff_musk										= 0x0204,//�������������ݣ�1Hz
	ID_aerial_robot_energy					= 0x0205,//���л���������״̬���ݣ�10Hz��ֻ�С������л��������ء�������
	ID_robot_hurt										= 0x0206,//�˺�״̬���ݣ��˺���������
	ID_shoot_data										= 0x0207,//ʵʱ������ݣ��ӵ��������
	ID_bullet_remaining							= 0x0208,//����ʣ�෢���������������л����ˣ��ڱ������ˡ����Լ�ICRA�����˷��ͣ�1Hz
	ID_rfid_status									= 0x0209,//������RFID״̬��1Hz
	
	ID_dart_client_directive        = 0x020A,//���ڻ����˿ͻ���ָ����, 10Hz
	
	ID_robot_interactive_header_data			= 0x0301,//�����˽������ݣ��������ͷ������������� 10Hz
	ID_controller_interactive_header_data = 0x0302,//�Զ���������������ݽӿڣ�ͨ�������ͻ��˴����������� 30Hz
	ID_map_interactive_header_data        = 0x0303,//�ͻ���С��ͼ�������ݣ������������͡���
	ID_keyboard_information               = 0x0304//���̡������Ϣ��ͨ������ͼ�����ڡ�������
};
//������ö��CMD_ID
//**************����ϵͳ������Ϣ�ĳ��ȣ���λ/�ֽڣ�**************************/
enum judge_data_length_t {
	/* Std */
	LEN_FRAME_HEAD 	                 = 5,	// ֡ͷ����
	LEN_CMD_ID 		                   = 2,	// �����볤��
	LEN_FRAME_TAIL 	                 = 2,	// ֡βCRC16
	/* Ext */
	// 0x000x
	LEN_GAME_STATUS 				         = 11,
	LEN_GAME_RESULT 				         = 1,
	LEN_GAME_ROBOT_HP 			         = 28,
	LEN_DART_STATUS					         = 3,
	LEN_ICRA_BUFF_DEBUFF_ZONE_STATUS = 11,//0x0005
	// 0x010x
	LEN_EVENT_DATA					         = 4,
	LEN_SUPPLY_PROJECTILE_ACTION	   = 4,//����������������������������������
	LEN_SUPPLY_PROJECTILE_BOOKING	   = 3,//�Կ���δ����
	LEN_REFEREE_WARNING				       = 2,
	LEN_DART_REMAINING_TIME		     	 = 1,//0x0105
	// 0x020x
	LEN_GAME_ROBOT_STATUS			       = 27,//15!!!!!!!!!!!!!!!!!!!!!!!!!!!
	LEN_POWER_HEAT_DATA 			       = 16,//��������������������
	LEN_GAME_ROBOT_POS				       = 16,
	LEN_BUFF_MASK		 				         = 1,
	LEN_AERIAL_ROBOT_ENERGY 	     	 = 1,//����������
	LEN_ROBOT_HURT				         	 = 1,
	LEN_SHOOT_DATA					         = 7,//��������
	LEN_BULLET_REMAINING	 		       = 6,//��������
	LEN_RFID_STATUS					         = 4,
	LEN_DART_CLIENT_DIRECTIVE        = 12,//0x020A
	// 0x030x
	//LEN_robot_interactive_header_data      = n,
	//LEN_controller_interactive_header_data = n,
	LEN_MAP_INTERACTIVE_HEADERDATA           = 15,
	LEN_KEYBOARD_INFORMATION                 = 12,//0x0304
};//��2-4

/*******************����ϵͳ��Ϣ��������**************************/


/* �Զ���֡ͷ */
typedef __packed struct
{
	uint8_t  SOF;						
	uint16_t DataLength;
	uint8_t  Seq;
	uint8_t  CRC8;
	
}FrameHeader;


typedef __packed struct
{
	uint8_t  sof;
	uint16_t data_length;
	uint8_t  seq;
	uint8_t  crc8;
} std_frame_header_t;//LEN_FRAME_HEAD

	/*******************************************************************************/

    
/* 
	ѧ�������˼�ͨ�� cmd_id 0x0301������ data_ID:0x0200~0x02FF
	�������� �����˼�ͨ�ţ�0x0301��
	����Ƶ�ʣ����������кϼƴ������� 5000 Byte�� �����з���Ƶ�ʷֱ𲻳���30Hz��
 * +------+------+-------------+------------------------------------+
 * | byte | size |    breif    |            note                    |
 * |offset|      |             |                                    |
 * +------+------+-------------+------------------------------------+
 * |  0   |  2   | 	 data_ID   | 0x0200~0x02FF,��������Щ ID ��ѡȡ |
 * |      |      |             | ����ID�����ɲ������Զ���           |
 * +------|------|-------------|------------------------------------|
 * |  2   |  2   | 	sender_ID  | ��ҪУ�鷢���ߵ� ID ��ȷ��					|
 * +------|------|-------------|------------------------------------|
 * |  4   |  2   | receiver_ID | ��ҪУ������ߵ� ID ��ȷ��					|
 * |      |      |             | ���粻�ܷ��͵��жԻ����˵�ID				| 
 * +------|------|-------------|------------------------------------|
 * |  6   |  n   | 		Data     | n ��ҪС�� 113 										|
 * +------+------+-------------+------------------------------------+
*/
/******************************�ͻ��˽�������**************************************/
#define INTERACT_DATA_LEN	113
typedef __packed struct //���ݶ����ݸ�ʽ
{ 
	uint16_t data_cmd_id;    
	uint16_t send_ID;    
	uint16_t receiver_ID; 
} ext_client_data_header_t; 
enum
{
	//0x200-0x02ff 	�����Զ������� ��ʽ  INTERACT_ID_XXXX
	INTERACT_ID_delete_graphic 			= 0x0100,	/*�ͻ���ɾ��ͼ��*/
	INTERACT_ID_draw_one_graphic 		= 0x0101,	/*�ͻ��˻���һ��ͼ��*/
	INTERACT_ID_draw_two_graphic 		= 0x0102,	/*�ͻ��˻���2��ͼ��*/
	INTERACT_ID_draw_five_graphic 	= 0x0103,	/*�ͻ��˻���5��ͼ��*/
	INTERACT_ID_draw_seven_graphic 	= 0x0104,	/*�ͻ��˻���7��ͼ��*/
	INTERACT_ID_draw_char_graphic 	= 0x0110,	/*�ͻ��˻����ַ�ͼ��*/
	INTERACT_ID_bigbome_num					= 0x02ff
};
typedef __packed struct 
{ 
	uint8_t data[INTERACT_DATA_LEN]; //���ݶ�,n��ҪС��113
} robot_interactive_data_t;
//��λ���ֽڣ�
enum
{
	LEN_INTERACT_delete_graphic     = 8,  //ɾ��ͼ�� 2(��������ID)+2(������ID)+2��������ID��+2���������ݣ�  
	LEN_INTERACT_draw_one_graphic   = 21, // ����2+2+2+15
	LEN_INTERACT_draw_two_graphic   = 36, //6+15*2
	LEN_INTERACT_draw_five_graphic  = 81, //6+15*5
	LEN_INTERACT_draw_seven_graphic = 111,//6+15*7
	LEN_INTERACT_draw_char_graphic  = 51, //6+15+30���ַ������ݣ�
};
//****************************��ͼ�����ݶ�����****************************/
typedef __packed struct//ͼ��
{                          
	uint8_t graphic_name[3]; 
	uint32_t operate_tpye:3; 
	uint32_t graphic_tpye:3; //ֱ��  ����  ��Բ  ��Բ  Բ��  ����  ����  �ַ�
	uint32_t layer:4;        
	uint32_t color:4;        
	uint32_t start_angle:9;  //��    ��    ��    ��    �Ƕ�  ��С  ��С  ��С
	uint32_t end_angle:9;    //��    ��    ��    ��          λ��  ��    ����
	uint32_t width:10;       
	uint32_t start_x:11;     //���  ���  Բ��  Բ��  Բ��  ���  ���  ���
	uint32_t start_y:11;     //
	uint32_t radius:10;      //��    ��    �뾶  ��    ��    ��    ��    ��
	uint32_t end_x:11;       //�յ�  �Զ�  ��    ����  ����  ��    ��    ��
	uint32_t end_y:11;       //                              ��    ��    ��
} graphic_data_struct_t;
typedef __packed struct//������
{                          
	uint8_t graphic_name[3]; 
	uint32_t operate_tpye:3; 
	uint32_t graphic_tpye:3; 
	uint32_t layer:4;        
	uint32_t color:4;        
	uint32_t start_angle:9;  
	uint32_t end_angle:9;    
	uint32_t width:10;       
	uint32_t start_x:11;    
	uint32_t start_y:11;     
  int number;       
} Float_data_struct_t;
typedef __packed struct//������
{                          
	uint8_t graphic_name[3]; 
	uint32_t operate_tpye:3; 
	uint32_t graphic_tpye:3; 
	uint32_t layer:4;        
	uint32_t color:4;        
	uint32_t start_angle:9;  
	uint32_t end_angle:9;    
	uint32_t width:10;       
	uint32_t start_x:11;    
	uint32_t start_y:11;     
  int number;       
} Int_data_struct_t;
/* data_ID: 0X0100  Byte:  2	    �ͻ���ɾ��ͼ��*/
typedef __packed struct
{
	uint8_t operate_type; 
	uint8_t layer;//ͼ������0~9
}ext_client_custom_graphic_delete_t;
typedef enum
{
	NONE_delete    = 0,
	GRAPHIC_delete = 1,
	ALL_delete     = 2
}delete_Graphic_Operate;//ext_client_custom_graphic_delete_t��uint8_t operate_type
/*ͼ��ɾ������*/

//bit 0-2
typedef enum
{
	NONE   = 0,/*�ղ���*/
	ADD    = 1,/*����ͼ��*/
	MODIFY = 2,/*�޸�ͼ��*/
	DELETE = 3,/*ɾ��ͼ��*/
}Graphic_Operate;//graphic_data_struct_t��uint32_t operate_tpye
/*ͼ�����*/
//bit3-5
typedef enum
{
	LINE      = 0,//ֱ��
	RECTANGLE = 1,//����
	CIRCLE    = 2,//��Բ
	OVAL      = 3,//��Բ
	ARC       = 4,//Բ��
	INT    	  = 5,//������
	FLOAT     = 6,//������
	CHAR      = 7 //�ַ�
}Graphic_Type;
/*ͼ������*/
//bit 6-9ͼ���� ���Ϊ9����С0
//bit 10-13��ɫ
typedef enum
{
	RED_BLUE  = 0,//������ɫ	
	YELLOW    = 1,
	GREEN     = 2,
	ORANGE    = 3,
	FUCHSIA   = 4,	/*�Ϻ�ɫ*/
	PINK      = 5,
	CYAN_BLUE = 6,	/*��ɫ*/
	BLACK     = 7,
	WHITE     = 8
}Graphic_Color;
/*ͼ����ɫ����*/
//bit 14-31 �Ƕ� [0,360]
/**********************************�ͻ��˻�ͼ************************************************/
//ɾ��ͼ��

typedef __packed struct
{
	std_frame_header_t txFrameHeader;			
	uint16_t  CmdID;										
	ext_client_data_header_t   dataFrameHeader;
	ext_client_custom_graphic_delete_t clientData;		
	uint16_t	FrameTail;								
}ext_deleteLayer_data_t;

//���ַ���
typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct;
	uint8_t data[30];
} ext_client_string_t;

typedef __packed struct
{
	std_frame_header_t txFrameHeader;			//֡ͷ
	uint16_t  CmdID;										//������
	ext_client_data_header_t   dataFrameHeader;//���ݶ�ͷ�ṹ
	ext_client_string_t clientData;//���ݶ�
	uint16_t	FrameTail;								//֡β
}ext_charstring_data_t;
//������ͼ
typedef __packed struct
{
	std_frame_header_t txFrameHeader;			//֡ͷ
	uint16_t  CmdID;										//������
	ext_client_data_header_t   dataFrameHeader;//���ݶ�ͷ�ṹ
	graphic_data_struct_t clientData;		//���ݶ�
	uint16_t	FrameTail;								//֡β
}ext_graphic_one_data_t;
typedef __packed struct
{
	std_frame_header_t txFrameHeader;			
	uint16_t  CmdID;										
	ext_client_data_header_t   dataFrameHeader;
	graphic_data_struct_t clientData[2];		
	uint16_t	FrameTail;
	
}ext_graphic_two_data_t;
typedef __packed struct
{
	std_frame_header_t txFrameHeader;			
	uint16_t  CmdID;										
	ext_client_data_header_t   dataFrameHeader;
	graphic_data_struct_t clientData[5];		
	uint16_t	FrameTail;								
}ext_graphic_five_data_t;
typedef __packed struct
{
	std_frame_header_t txFrameHeader;			
	uint16_t  CmdID;										
	ext_client_data_header_t   dataFrameHeader;
	graphic_data_struct_t clientData[7];		
	uint16_t	FrameTail;								
}ext_graphic_seven_data_t;
//���Ƹ�����
typedef __packed struct
{
	std_frame_header_t txFrameHeader;			
	uint16_t  CmdID;										
	ext_client_data_header_t   dataFrameHeader;
	Float_data_struct_t clientData[2];		
	uint16_t	FrameTail;								
}ext_float_two_data_t;

typedef __packed struct
{
	std_frame_header_t txFrameHeader;			
	uint16_t  CmdID;										
	ext_client_data_header_t   dataFrameHeader;
	Float_data_struct_t clientData[7];		
	uint16_t	FrameTail;								
}ext_float_seven_data_t;
//��������
typedef __packed struct
{
	std_frame_header_t txFrameHeader;			
	uint16_t  CmdID;										
	ext_client_data_header_t   dataFrameHeader;
	Int_data_struct_t clientData[2];		
	uint16_t	FrameTail;								
}ext_int_two_data_t;
typedef __packed struct
{
	std_frame_header_t txFrameHeader;			
	uint16_t  CmdID;										
	ext_client_data_header_t   dataFrameHeader;
	Int_data_struct_t clientData[7];		
	uint16_t	FrameTail;								
}ext_int_seven_data_t;
 #endif
typedef __packed struct Client_Slave_Flag
{
	uint8_t global_fiction;
	uint8_t global_clip;
	uint8_t global_spin;
	uint8_t global_auto_aim;
	uint8_t global_auto_arrive;
	uint8_t global_twist;
	uint8_t global_anti_top;
	uint8_t shift_rush;
	uint8_t move_mode;
  uint8_t barrel_type;
	uint8_t user1;
	
	int16_t yaw;
}Client_Slave_Flag;



extern uint8_t update_figure_flag,update_aim_flag,update_float_flag,update_supercapacitor_flag,update_int_flag,update_gimbalangle_flag;//�ͻ��˸���flag
extern Client_Slave_Flag Slaver_flag;

//void Determine_ID(void);

void DJI_Judge_Init(void);
void Client_graphic_Init(void);
void Client_graphic_delete_update(uint8_t delete_layer);//ɾ��ͼ����Ϣ
void Client_graphic_Info_update(void);
void Client_aim_update(void);//׼��
void Client_supercapacitor_update(void);//��������
void Client_gimbalangle_figure_update(void);//��̨YAW�Ƕ�ͼ�λ���ʾ
void Client_gimbal_angle_update(void);//����Ƕ�
void Client_bullet_int_update(void);//������Ϣ
void Client_aim_line(void);//Ӣ��

void Client_info_update(void);//��Ϣ����
void PITCH_YAW_Analyze(uint8_t *buf);//pitch��yaw����
// ����ϵͳ�ķ�������
void JUDGE_Analyze(volatile uint8_t *databuff, uint8_t lenth);
void Client_task(void);


#endif
