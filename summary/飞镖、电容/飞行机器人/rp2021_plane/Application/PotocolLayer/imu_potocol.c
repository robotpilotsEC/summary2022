/**
 * @file        imu_potocol.c
 * @author      RobotPilots@2020
 * @Version     V1.0
 * @date        9-September-2020
 * @brief       Imu Potocol.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "imu_potocol.h"

#include "bmi.h"

#include "imu_sensor.h"
#include "rp_math.h"
#include "drv_haltick.h"
#include "usart.h"
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/*
    rate_yaw ��ʱ��Ϊ��
    yaw ��ʱ����������
    rate_pitch ��ʱ��Ϊ��
    pitch ��ʱ���С������
    rate_roll ��ʱ��Ϊ��
    roll ��ʱ���С������
*/

int16_t IMUData[3];

unsigned char Sum_Get(char *dat,char len)
{
	char i,sum=0;
	for(i=0; i<len;i++)
	{
		sum += *dat++;
	}
	return sum;
}

void IMU3Ddisplay(imu_sensor_t *imu_sen){
    short temp[3];
	unsigned char cTemp[32], Index = 0;
	char i = 0;
    
    IMUData[0] = imu_sen->info->pitch / 0.006f;
    IMUData[1] = imu_sen->info->yaw / 0.006f;
    IMUData[2] = imu_sen->info->roll / 0.006f;
	
	temp[0] = IMUData[0];
	temp[1] = IMUData[1];
	temp[2] = IMUData[2];

	cTemp[Index++] = 0xaa;
	cTemp[Index++] = 0x55;
	cTemp[Index++] = 0x03;
	cTemp[Index++] = 0x06;
	
	for(i = 0; i < 3;  i ++){
		cTemp[Index++] = (temp[i] >> 8);
		cTemp[Index++] = (temp[i]);
	}

	cTemp[Index]= Sum_Get((char*)cTemp, Index);

	HAL_UART_Transmit(&huart1, cTemp, Index+1, 50);
}

short gyrox, gyroy, gyroz;
short accx, accy, accz;
void imu_sensor_update(imu_sensor_t *imu_sen)
{
    imu_sensor_info_t *imu_info = imu_sen->info;
	
    BMI_Get_RawData(&gyrox, &gyroy, &gyroz, &accx, &accy, &accz);
    BMI_Get_EulerAngle(&imu_info->pitch, &imu_info->roll, &imu_info->yaw, &gyrox, &gyroy, &gyroz, &accx, &accy, &accz);
    
	imu_info->rate_pitch = gyroy; // ����roll���ٶ� gyrox;
	imu_info->rate_roll = gyrox; // ����pitch���ٶ� gyroy;
	imu_info->rate_yaw = gyroz;
    
//    IMU3Ddisplay(imu_sen);
    
	imu_sen->check(imu_sen);
}

//int8_t imu_init_errno;
void imu_sensor_init(imu_sensor_t *imu_sen)
{
    int8_t rslt;
	uint32_t tickstart = HAL_GetTick();
	imu_sensor_info_t *imu_info = imu_sen->info;
	
	imu_sen->errno = NONE_ERR;

    rslt = BMI_Init();
	while(rslt) {
        // �����ʼ��ʧ�������³�ʼ��
        imu_sen->errno = DEV_INIT_ERR;
        rslt = BMI_Init();
    }
    //imu_init_errno = rslt;
        
	for(uint16_t i=0; i<1000; i++) {
        BMI_Get_RawData(&imu_info->rate_roll, &imu_info->rate_pitch, &imu_info->rate_yaw, &accx, &accy, &accz);
        BMI_Get_EulerAngle(&imu_info->pitch, &imu_info->roll, &imu_info->yaw, &imu_info->rate_roll, &imu_info->rate_pitch, &imu_info->rate_yaw, &accx, &accy, &accz);
		imu_info->rate_pitch_offset += imu_info->rate_pitch;
		imu_info->rate_yaw_offset += imu_info->rate_yaw;
        delay_ms(1);
	}
    /**
        @note
        ����ϵ��ʱ����̨�˶����ᵼ�¼�������ľ�̬ƫ����ֵ�������ÿ���ϵ��ʱ�򣬾�̬ƫ���
        ��𲻴�Ļ�������ֱ�Ӹ���һ���̶�ֵ�����ߣ�����Լ��������ƫ��ֵ���жϵȡ�
    */
	imu_info->rate_pitch_offset /= 1000.f;
	imu_info->rate_yaw_offset /= 1000.f;
    
    extern float Kp;
    Kp = 0.1f;

	if(imu_sen->id != DEV_ID_IMU)
		imu_sen->errno = DEV_ID_ERR;
}
