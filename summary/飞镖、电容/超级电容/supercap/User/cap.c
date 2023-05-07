#include "cap.h"
#include "adc.h"
#include "string.h"
#include "stdio.h"
#include "hrtim.h"
#include "tim.h"
#include "opamp.h"
#include "gpio.h"
#include "can.h"
#include "oled.h"
#include "kalman.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"
#include "flash.h"

#define abs(x) 					((x)>0? (x):(-(x)))
#define max(a,b)                ((a)>(b) ? (a):(b))
#define min(a,b)                ((a)<(b) ? (a):(b))
#define range(a,b,c)            (min(max(a,b),c))   //��Χ���ƣ�aΪĿ��ֵ��bΪ��Сֵ��cΪ���ֵ

#define rank_bat_v    0
#define rank_bat_i    3
#define rank_cap_v    1
#define rank_cap_i    4
#define rank_chas_v   5
#define rank_chas_i   2

#define cap_vol_min_limit 8.f
#define chas_vol_max_limit 30.f

float cap_vol_max_limit = 23.5f;
float bat_power = 0;
float cap_power = 0;
float chas_power = 0;
uint32_t count = 0;
float efficiency_sec = 0;   //һ���ڵĵ�ǰЧ��
float efficiency_rec = 0;   //��¼ʱ���ڵ�Ч��
float capacity = 0;
float resistance = 0;
int16_t can_send_data[4] = {0};
uint32_t outtime[5] = {0,0,0,0,1000};
uint8_t pwm_switch = 1;     //������debug�п���pwm����
uint8_t adjust_switch = 0;     //����ģʽ���أ�1Ϊ���������ԣ�2Ϊ���ʻ����ԣ�3Ϊ�������ݵ���
uint8_t key_press = 0;
uint8_t display_mode = 0;   //ģʽ0����ʾ������Ϣ��ģʽ1����ʾ����������Ϣ
float powerin_rec = 0;
float powerout_rec = 0;

extKalman_t p_bat_v;
extKalman_t p_bat_i;
extKalman_t p_cap_v;
extKalman_t p_cap_i;
extKalman_t p_chas_v;
extKalman_t p_chas_i;
extKalman_t p_bat_i_err;
extKalman_t p_bat_v_err;

power_adjust_t power_adjust;
cap_vol_max_adjust_t cap_vol_max_adjust;
receive_data_t receive_data;
cap_state_t cap_state;
load_t load;

extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;

adc_data_t adc_data = {
//2��
//    .num = 2,
//    .bat_v_m = 1.f,
//    .bat_i_m = 0.9999f,
//    .cap_v_m = 1.f,
//    .cap_i_m = 0.8483f,
//    .chas_v_m = 1.f,
//    .chas_i_m = 0.9913f,
//    .bat_v_a = 0.0f,
//    .bat_i_a = 0.1565f,
//    .cap_v_a = 0.0f,
//    .cap_i_a = -0.0538f,
//    .chas_v_a = 0.0f,
//    .chas_i_a = -0.3622f,  

//0�� ok-5.16
//    .num = 0,
//    .bat_v_m = 1.0f,
//    .bat_i_m = 1.0069f,
//    .cap_v_m = 0.9939f,
//    .cap_i_m = 0.8347f,
//    .chas_v_m = 1.0f,
//    .chas_i_m = 1.0146f,
//    .bat_v_a = 0.0f,
//    .bat_i_a = 0.2411f,
//    .cap_v_a = 0.058f,
//    .cap_i_a = -0.1069f,
//    .chas_v_a = 0.0f,
//    .chas_i_a = -0.5483f,
    
//3
//    .num = 3,
//    .bat_v_m = 1.f,
//    .bat_i_m = 1.0058f,
//    .cap_v_m = 0.9929f,
//    .cap_i_m = 0.8328f,
//    .chas_v_m = 1.f,
//    .chas_i_m = 0.9982f,
//    .bat_v_a = 0.f,
//    .bat_i_a = 0.1838f,
//    .cap_v_a = -0.0811f,
//    .cap_i_a = -0.1598f,
//    .chas_v_a = 0.f,
//    .chas_i_a = -0.5685f,

//5
//    .num = 5,
//    .bat_v_m = 1.f,
//    .bat_i_m = 1.0062f,
//    .cap_v_m = 0.9846f,
//    .cap_i_m = 0.8387f,
//    .chas_v_m = 1.f,
//    .chas_i_m = 0.9829f,
//    .bat_v_a = 0.05f,
//    .bat_i_a = 0.4452f,
//    .cap_v_a = 0.025f,
//    .cap_i_a = 0.1259f,
//    .chas_v_a = -0.3f,
//    .chas_i_a = -0.0297f,

//4
//    .num = 4,
//    .bat_v_m = 1.0076f,
//    .bat_i_m = 1.0154f,
//    .cap_v_m = 1.f,
//    .cap_i_m = 0.8398f,
//    .chas_v_m = 1.f,
//    .chas_i_m = 0.9891f,
//    .bat_v_a = 0.0185f,
//    .bat_i_a = 0.1313f,
//    .cap_v_a = 0.f,
//    .cap_i_a = 0.0021f,
//    .chas_v_a = 0.f,
//    .chas_i_a = -0.1848f,

//1 chas_i������
    
//8 
//    .num = 8,
//    .bat_v_m = 1.f,
//    .bat_i_m = 0.9993f,
//    .cap_v_m = 0.9941f,
//    .cap_i_m = 0.8219f,
//    .chas_v_m = 1.f,
//    .chas_i_m = 0.9924f,
//    .bat_v_a = 0.f,
//    .bat_i_a = 0.0867f,
//    .cap_v_a = -0.0328f,
//    .cap_i_a = 0.1556f,
//    .chas_v_a = -0.23f,
//    .chas_i_a = -0.0966f,

//9
//    .num = 9,
//    .bat_v_m = 1.f,
//    .bat_i_m = 1.0138f,
//    .cap_v_m = 1.0035f,
//    .cap_i_m = 0.8286f,
//    .chas_v_m = 1.f,
//    .chas_i_m = 0.9982f,
//    .bat_v_a = 0.f,
//    .bat_i_a = 0.0577f,
//    .cap_v_a = 0.0251f,
//    .cap_i_a = 0.1179f,
//    .chas_v_a = 0.f,
//    .chas_i_a = -0.3596f,
    
//�̰� 8
//    .num = 8,
//    .bat_v_m = 1.f,
//    .bat_i_m = 0.993f,
//    .cap_v_m = 0.9867f,
//    .cap_i_m = 0.8563f,
//    .chas_v_m = 1.f,
//    .chas_i_m = 0.9848f,
//    .bat_v_a = 0.f,
//    .bat_i_a = -0.0069f,
//    .cap_v_a = -0.0061f,
//    .cap_i_a = -0.1663f,
//    .chas_v_a = 0.f,
//    .chas_i_a = -0.1502f,

//10
//    .num = 10,
//    .bat_v_m = 1.f,
//    .bat_i_m = 1.0088f,
//    .cap_v_m = 1.f,
//    .cap_i_m = 0.8526f,
//    .chas_v_m = 1.f,
//    .chas_i_m = 0.978f,
//    .bat_v_a = 0.f,
//    .bat_i_a = 0.1609f,
//    .cap_v_a = 0.f,
//    .cap_i_a = -0.078f,
//    .chas_v_a = 0.f,
//    .chas_i_a = -0.0363f,
    
//11
//    .num = 11,
//    .bat_v_m = 1.f,
//    .bat_i_m = 1.0206f,
//    .cap_v_m = 1.f,
//    .cap_i_m = 0.8274f,
//    .chas_v_m = 1.f,
//    .chas_i_m = 0.9905f,
//    .bat_v_a = 0.f,
//    .bat_i_a = 0.0682f,
//    .cap_v_a = 0.f,
//    .cap_i_a = -0.0185f,
//    .chas_v_a = 0.f,
//    .chas_i_a = -0.1025f,
    
//12
//    .num = 12,
//    .bat_v_m = 1.f,
//    .bat_i_m = 0.996f,
//    .cap_v_m = 1.f,
//    .cap_i_m = 0.7919f,
//    .chas_v_m = 1.f,
//    .chas_i_m = 0.9886f,
//    .bat_v_a = 0.f,
//    .bat_i_a = 0.2372f,
//    .cap_v_a = 0.f,
//    .cap_i_a = -0.0865f,
//    .chas_v_a = 0.f,
//    .chas_i_a = -0.35f,

//13
//    .num = 13,
//    .bat_v_m = 1.f,
//    .bat_i_m = 0.9991f,
//    .cap_v_m = 1.f,
//    .cap_i_m = 0.8273f,
//    .chas_v_m = 1.f,
//    .chas_i_m = 0.9948f,
//    .bat_v_a = 0.f,
//    .bat_i_a = -0.0818f,
//    .cap_v_a = 0.f,
//    .cap_i_a = -0.0595f,
//    .chas_v_a = 0.f,
//    .chas_i_a = -0.3047f,

//14
//    .num = 14,
//    .bat_v_m = 1.f,
//    .bat_i_m = 1.014f,
//    .cap_v_m = 1.f,
//    .cap_i_m = 0.8189f,
//    .chas_v_m = 1.f,
//    .chas_i_m = 0.9766f,
//    .bat_v_a = 0.f,
//    .bat_i_a = 0.1135f,
//    .cap_v_a = 0.f,
//    .cap_i_a = 0.1425f,
//    .chas_v_a = 0.f,
//    .chas_i_a = 0.0772f,
    
//��ʼ
    .num = 0,
    .bat_v_m = 1.f,
    .bat_i_m = 1.f,
    .cap_v_m = 1.f,
    .cap_i_m = 1.f,
    .chas_v_m = 1.f,
    .chas_i_m = 1.f,
    .bat_v_a = 0.f,
    .bat_i_a = 0.f,
    .cap_v_a = 0.f,
    .cap_i_a = 0.f,
    .chas_v_a = 0.f,
    .chas_i_a = 0.f,
};

//���ռ�ձȺ���Сռ�ձȿ���һ���̶��Ϸ�ֹ�������
pid_cap_i_t pid_cap_i = {
    .kp = 0,
    .ki = 0.001,
    .kd = 0,
    .integralmax = 1100,
    .integralmin = 150,
    .deadarea = 0,
    .OutDutyCycleMax = 1.1,
    .OutDutyCycleMin = 0.15,
    .PidSwitch = 0,
    .Filter_fac = 0,
};

//�����������������������ŵ������ע��������޺͵������Ƶı�����ϵ��Ҫ��ͬʱ��
pid_bat_power_t pid_bat_power = {
    .kp = 0.05,
    .ki = 0.005,
    .kd = 0,
    .integralmax = 2400,
    .integralmin = -2600,
    .deadarea = 0,
    .OutCurrentMax = 12,
    .OutCurrentMin = -13,
    .PidSwitch = 0,
    .Filter_fac = 0,
};

pid_powerbuffer_t pid_powerbuffer = {
    .kp = 1,
    .ki = 0.004,
    .kd = 0,
    .SetBuffer = 50,
    .deadarea = 0,
    .PidSwitch = 0,
    .Filter_fac = 0,
};

void adc_solve()
{
//    adc_data.unkal_bat_v = (float)adc_data.adc_list_record[0] / 4096 * 3.3f * 15 * adc_data.bat_v_m + adc_data.bat_v_a;
//    adc_data.unkal_bat_i = ((float)adc_data.adc_list_record[3] / 4096 * 3.3f - 1.65f) / 0.075f * adc_data.bat_i_m + adc_data.bat_i_a;
//    adc_data.unkal_cap_v = (float)adc_data.adc_list_record[1] / 4096 * 3.3f * 15 * adc_data.cap_v_m + adc_data.cap_v_a;
//    adc_data.unkal_cap_i = -(((float)adc_data.adc_list_record[4] / 4096 * 3.3f - 1.65f) / 0.075f) * adc_data.cap_i_m + adc_data.cap_i_a;
//    adc_data.unkal_chas_v = (float)adc_data.adc_list_record[5] / 4096 * 3.3f * 15 * adc_data.chas_v_m + adc_data.chas_v_a;
//    adc_data.unkal_chas_i = -((float)adc_data.adc_list_record[2] / 4096 * 3.3f - 1.65f) / 0.062f * adc_data.chas_i_m + adc_data.chas_i_a;

    adc_data.bat_v_unmix = (float)adc_data.adc_list_kal[0] / 4096 * 3.3f * 15 * adc_data.bat_v_m + adc_data.bat_v_a;
    adc_data.bat_i_unmix = ((float)adc_data.adc_list_kal[3] / 4096 * 3.3f - 1.65f) / 0.075f * adc_data.bat_i_m + adc_data.bat_i_a;
    adc_data.bat_v = adc_data.bat_v_unmix + adc_data.bat_v_err;
    adc_data.bat_i = adc_data.bat_i_unmix + adc_data.bat_i_err;
    adc_data.cap_v = (float)adc_data.adc_list_kal[1] / 4096 * 3.3f * 15 * adc_data.cap_v_m + adc_data.cap_v_a;
    adc_data.chas_i = -((float)adc_data.adc_list_kal[2] / 4096 * 3.3f - 1.65f) / 0.062f * adc_data.chas_i_m + adc_data.chas_i_a;
    //    adc_data.cap_i = -(((float)adc_data.adc_list_kal[4] / 4096 * 3.3f - 1.65f) / 0.075f) * adc_data.cap_i_m + adc_data.cap_i_a;  //�ڶ�ʱ���жϼ���
    adc_data.chas_v = (float)adc_data.adc_list_kal[5] / 4096 * 3.3f * 15 * adc_data.chas_v_m + adc_data.chas_v_a;
}

uint32_t count_hdma_adc1;
uint32_t count_hdma_adc2;
void hdma_adc1_cplt(struct __DMA_HandleTypeDef * hdma)
{
//    static uint32_t load_tick;
    if(hdma == &hdma_adc1)
    {
//        load_tick = get_tickus();
//        count_hdma_adc1++;
        adc_data.adc_list_kal[0] = KalmanFilter(&p_bat_v, (float)adc_data.adc_list_record[0]);
        adc_data.adc_list_kal[1] = KalmanFilter(&p_cap_v, (float)adc_data.adc_list_record[1]);
        adc_data.adc_list_kal[2] = KalmanFilter(&p_chas_i, (float)adc_data.adc_list_record[2]);
//        load.dma1 = get_tickus() - load_tick;
//        load.dma1_max = max(load.dma1, load.dma1_max);
    }
}
void hdma_adc2_cplt(struct __DMA_HandleTypeDef * hdma)
{
//    static uint32_t load_tick;
    if(hdma == &hdma_adc2)
    {
//        load_tick = get_tickus();
//        count_hdma_adc2++;
        adc_data.adc_list_kal[3] = KalmanFilter(&p_bat_i, (float)adc_data.adc_list_record[3]);
        adc_data.adc_list_kal[4] = KalmanFilter(&p_cap_i, (float)adc_data.adc_list_record[4]);
        adc_data.adc_list_kal[5] = KalmanFilter(&p_chas_v, (float)adc_data.adc_list_record[5]);
//        load.dma2 = get_tickus() - load_tick;
//        load.dma2_max = max(load.dma2, load.dma2_max);
    }
}

void cap_pid_i()
{
    //�ж��Ƿ����и�pid
    if(pid_cap_i.PidSwitch == 0){
        pid_cap_i.integral = adc_data.cap_v / (adc_data.bat_v * pid_cap_i.ki);  //�����ŵĻ����ۼ�ֵ���������ĵ�������
        return;
    }
    
    if(adjust_switch != 1)
    {
        pid_cap_i.SetI = pid_bat_power.OutCurrent;  
    }
    pid_cap_i.ActualI = adc_data.cap_i;
    
    //����ƫ��
    pid_cap_i.err = pid_cap_i.SetI - pid_cap_i.ActualI;

    //��������
//    if(abs(pid_cap_i.err) < pid_cap_i.deadarea)
//    {
//        return;
//    }
    
    //�����ۼ�
    pid_cap_i.integral += pid_cap_i.err;
    
    if(pid_cap_i.integral > pid_cap_i.integralmax)
    {
        pid_cap_i.integral = pid_cap_i.integralmax;
    }
    else if(pid_cap_i.integral < pid_cap_i.integralmin)
    {
        pid_cap_i.integral = pid_cap_i.integralmin;
    }
    
    //�������
    pid_cap_i.pout = pid_cap_i.kp * pid_cap_i.err;
    pid_cap_i.iout = pid_cap_i.ki * pid_cap_i.integral;
    pid_cap_i.dout = pid_cap_i.kd * (pid_cap_i.err - pid_cap_i.err_last);
    
    float temp = pid_cap_i.pout + pid_cap_i.iout + pid_cap_i.dout;
    
    //�������������
    if(temp > pid_cap_i.OutDutyCycleMax)
    {
        pid_cap_i.OutDutyCycle = pid_cap_i.OutDutyCycleMax;
        pid_cap_i.overflag = 1;
    }
    else if(temp < pid_cap_i.OutDutyCycleMin)
    {
        pid_cap_i.OutDutyCycle = pid_cap_i.OutDutyCycleMin;
        pid_cap_i.overflag = 1;
    }
    else
    {
        pid_cap_i.OutDutyCycle = temp;
        pid_cap_i.overflag = 0;
    }
    
    //һ���ͺ��˲���������
//    if(pid_cap_i.Filter_fac > 0.5f)
//    {
//        pid_cap_i.OutDutyCycle = pid_cap_i.LastOutDutyCycle * pid_cap_i.Filter_fac \
//                                   + pid_cap_i.OutDutyCycle * (1.0f - pid_cap_i.Filter_fac);
//        pid_cap_i.LastOutDutyCycle = pid_cap_i.OutDutyCycle;
//    }
    
    //��¼����ƫ��
    pid_cap_i.err_last = pid_cap_i.err;
    
    HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2|HRTIM_OUTPUT_TD1|HRTIM_OUTPUT_TD2); //ͨ����
//    count ++;   //������pid���д������������Ӧ��1������10000
}

void cap_pid_bat_power()
{
    static uint8_t flag = 0;
    static uint16_t timetick = 0;
    //�ж��Ƿ����и�pid
    if(pid_bat_power.PidSwitch == 0)
    {
        pid_bat_power.integral = 0;
        pid_bat_power.OutCurrent = 0;
        pid_bat_power.overflag = 1;
        flag = 0;
        timetick = 0;
        return;
    }
    else if(pid_bat_power.PidSwitch == 1 && flag <= 1)
    {
        //������
        if(timetick < 500)
        {
            if(abs(adc_data.cap_i) < 2.5f)
            {
                timetick ++;
            }
            pid_bat_power.OutCurrent = 0;
            pid_bat_power.integral = 0;
            pid_bat_power.overflag = 1;
            return;
        }
        else if(timetick < 1000)
        {
            flag = 1;
            timetick ++;
        }
        else
        {
            flag = 2;
            timetick = 0;
        }
    }
    
    if(adjust_switch != 2)
    {
        pid_bat_power.SetPower = pid_powerbuffer.OutPower;
    }
    pid_bat_power.ActualPower = adc_data.bat_v * adc_data.bat_i;
    
    //����ƫ��
    pid_bat_power.err = pid_bat_power.SetPower - pid_bat_power.ActualPower;

    //��������
//    if(abs(pid_bat_power.err) < pid_bat_power.deadarea)
//    {
//        return;
//    }
    
    //�����ۼ�
    pid_bat_power.integral += pid_bat_power.err;
    
    //���ƻ���������
    if(pid_bat_power.integral > pid_bat_power.integralmax)
    {
        pid_bat_power.integral = pid_bat_power.integralmax;
    }
    else if(pid_bat_power.integral < pid_bat_power.integralmin)
    {
        pid_bat_power.integral = pid_bat_power.integralmin;
    }
    
    //�������
    pid_bat_power.pout = pid_bat_power.kp * pid_bat_power.err;
    pid_bat_power.iout = pid_bat_power.ki * pid_bat_power.integral;
    pid_bat_power.dout = pid_bat_power.kd * (pid_bat_power.err - pid_bat_power.err_last);
    
    float temp = pid_bat_power.pout + pid_bat_power.iout + pid_bat_power.dout;
    
    //ɲ�����ٻ�������
    if(adc_data.bat_v + 2 < adc_data.chas_v)
    {
        temp += adc_data.chas_v - adc_data.bat_v - 2;
    }
    float temp_max = min(pid_bat_power.OutCurrentMax, receive_data.input_power_limit / adc_data.cap_v);
    float temp_min = max(pid_bat_power.OutCurrentMin, -receive_data.output_power_limit / adc_data.cap_v);
    //�������������
    if(temp > temp_max)
    {
        pid_bat_power.OutCurrent = temp_max;
        pid_bat_power.overflag = 1;
    }
    else if(temp < temp_min)
    {
        pid_bat_power.OutCurrent = temp_min;
        pid_bat_power.overflag = 1;
    }
    else
    {
        pid_bat_power.OutCurrent = temp;
        pid_bat_power.overflag = 0;
    }
    //������
    if(flag == 1)
    {
        pid_bat_power.OutCurrent = (pid_bat_power.OutCurrent - adc_data.cap_i) * ((float)(timetick - 500) / 500) + adc_data.cap_i;
    }
    
    //���ƹ������
    if(adc_data.cap_v < cap_vol_min_limit + 2.0f && pid_bat_power.OutCurrent < 0)
    {
        pid_bat_power.OutCurrent = max(0.5f*max(adc_data.cap_v - cap_vol_min_limit, 0)*pid_bat_power.OutCurrentMin, pid_bat_power.OutCurrent);
        pid_bat_power.integral = (pid_bat_power.OutCurrent - pid_bat_power.pout) / pid_bat_power.ki;
        pid_bat_power.overflag = 1;
    }
    else if(adc_data.cap_v > cap_vol_max_limit - 2.0f && pid_bat_power.OutCurrent > 0)
    {
        pid_bat_power.OutCurrent = min(0.5f*max(cap_vol_max_limit - adc_data.cap_v, 0)*pid_bat_power.OutCurrentMax, pid_bat_power.OutCurrent);
        pid_bat_power.integral = (pid_bat_power.OutCurrent - pid_bat_power.pout) / pid_bat_power.ki;
        pid_bat_power.overflag = 1;
    }
    else if(adc_data.chas_v > adc_data.bat_v && pid_bat_power.OutCurrent < 0)
    {
        pid_bat_power.OutCurrent *= max(0.5f*(adc_data.bat_v + 2.0f - adc_data.chas_v), 0);
        pid_bat_power.integral = (pid_bat_power.OutCurrent - pid_bat_power.pout) / pid_bat_power.ki;
        pid_bat_power.overflag = 1;
    }
    
    
    //һ���ͺ��˲���������
//    if(pid_bat_power.Filter_fac > 0.5f)
//    {
//        pid_bat_power.OutCurrent = pid_bat_power.LastOutCurrent * pid_bat_power.Filter_fac \
//                                   + pid_bat_power.OutCurrent * (1.0f - pid_bat_power.Filter_fac);
//        pid_bat_power.LastOutCurrent = pid_bat_power.OutCurrent;
//    }
    
    //��¼����ƫ��
    pid_bat_power.err_last = pid_bat_power.err;
}

void cap_pid_powerbuffer()
{
    //ע������������pid�����̹��ʻ���Խ�����ϵͳ������ʾ�ҪԽ�󣬼���ƫ��ķ�ʽ�븺�����෴��
    //�ж��Ƿ����и�pid
    if(pid_powerbuffer.PidSwitch == 0){
        pid_powerbuffer.integral = 0;
        return;
    }
    
    //���ݲ���ϵͳ��Ϣ���ڹ������ƣ�ȡֵ��Χ�ڹ������ơ�5W����ֹ���ʲ���̫��
    //��Ϊ����ϵͳ��������0.1s��ˢ��һ�Σ��ı书�ʶ����������Ӱ�������ͺ��ԣ��������pid�ڵ��̹��ʿ��ٱ仯ʱЧ�����á�
    //����ȡֵ��ΧԽС��pid����ԽС�����Թ��ʲ�����׼ȷ��Ҫ����
    pid_powerbuffer.OutPowerMax = receive_data.chassis_power_limit + 5;
    pid_powerbuffer.OutPowerMin = receive_data.chassis_power_limit - 5;
    pid_powerbuffer.integralmax = pid_powerbuffer.OutPowerMax / pid_powerbuffer.ki;
    pid_powerbuffer.integralmin = pid_powerbuffer.OutPowerMin / pid_powerbuffer.ki;

    pid_powerbuffer.ActualBuffer = receive_data.chassis_power_buffer;
    
    //����ƫ��
    pid_powerbuffer.err = pid_powerbuffer.ActualBuffer - pid_powerbuffer.SetBuffer;
    
    if(pid_powerbuffer.err > 1)
    {
        pid_powerbuffer.err = pid_powerbuffer.err * pid_powerbuffer.err;
    }
    else if(pid_powerbuffer.err < -1)
    {
        pid_powerbuffer.err = -pid_powerbuffer.err * pid_powerbuffer.err;
    }

    //��������
    if(abs(pid_powerbuffer.err) < pid_powerbuffer.deadarea)
    {
        return;
    }
    
    //�����ۼ�
    pid_powerbuffer.integral += pid_powerbuffer.err;
    
    //���ƻ���������
    if(pid_powerbuffer.integral > pid_powerbuffer.integralmax)
    {
        pid_powerbuffer.integral = pid_powerbuffer.integralmax;
    }
    else if(pid_powerbuffer.integral < pid_powerbuffer.integralmin)
    {
        pid_powerbuffer.integral = pid_powerbuffer.integralmin;
    }
    
    //�������
    pid_powerbuffer.pout = pid_powerbuffer.kp * pid_powerbuffer.err;
    pid_powerbuffer.iout = pid_powerbuffer.ki * pid_powerbuffer.integral;
    pid_powerbuffer.dout = pid_powerbuffer.kd * (pid_powerbuffer.err - pid_powerbuffer.err_last);
    
    float temp = pid_powerbuffer.pout + pid_powerbuffer.iout + pid_powerbuffer.dout;
    
    //�������������
    if(temp > pid_powerbuffer.OutPowerMax)pid_powerbuffer.OutPower = pid_powerbuffer.OutPowerMax;
    else if(temp < pid_powerbuffer.OutPowerMin)pid_powerbuffer.OutPower = pid_powerbuffer.OutPowerMin;
    else pid_powerbuffer.OutPower = temp;
    
    //�͹��ʻ���ʱ���ٻָ�����ɲ������������������
    if(pid_powerbuffer.ActualBuffer < 42 && adc_data.bat_v + 2 > adc_data.chas_v)
    {
        pid_powerbuffer.OutPower *= max(0.05f * (pid_powerbuffer.ActualBuffer - 22), 0);
    }
    
    //һ���ͺ��˲���������
//    if(pid_powerbuffer.Filter_fac > 0.5f)
//    {
//        pid_powerbuffer.OutPower = pid_powerbuffer.LastOutPower * pid_powerbuffer.Filter_fac \
//                                   + pid_powerbuffer.OutPower * (1.0f - pid_powerbuffer.Filter_fac);
//        pid_powerbuffer.LastOutPower = pid_powerbuffer.OutPower;
//    }
    
    //��¼����ƫ��
    pid_powerbuffer.err_last = pid_powerbuffer.err;
}

void err_adjust()
{
    if(pid_cap_i.overflag == 0 && pid_bat_power.overflag == 0)
    {
        power_adjust.downtime = (receive_data.chassis_power_buffer < 46) ? ++power_adjust.downtime : 0;
        power_adjust.uptime = (receive_data.chassis_power_buffer == 60) ? ++power_adjust.uptime : 0;
        if(power_adjust.downtime > 5000)
        {
            power_adjust.downtime = 2000;
            power_adjust.val -= 3;
        }
        else if(power_adjust.uptime > 5000)
        {
            power_adjust.uptime = 2000;
            power_adjust.val += 3;
        }
    }
    
    //�����������ѹ���޺�������ᵼ�·��ȣ���ÿ��������ĵ�ѹ���޶���ϸ΢���
    //���㷨��������Ӧ���ĵ�ѹ����
    cap_vol_max_adjust.time = (adc_data.cap_i > 0) ? ++cap_vol_max_adjust.time : 0;
    if(cap_vol_max_adjust.time2 < 500)
    {
        cap_vol_max_adjust.time2++;
        cap_vol_max_adjust.cap_i_sum += adc_data.cap_i;
    }
    else
    {
        cap_vol_max_adjust.time2 = 0;
        cap_vol_max_adjust.cap_i_sum_queue[cap_vol_max_adjust.queue_tail] = cap_vol_max_adjust.cap_i_sum;
        cap_vol_max_adjust.cap_v_queue[cap_vol_max_adjust.queue_tail] = adc_data.cap_v - 0.2f*adc_data.cap_i;
        
        if(cap_vol_max_adjust.time > 9999 \
           && (cap_vol_max_adjust.cap_i_sum_queue[cap_vol_max_adjust.queue_tail] \
               - cap_vol_max_adjust.cap_i_sum_queue[(cap_vol_max_adjust.queue_tail + 1) % 20] > 3000))
        {
            //��������ʱ�����10����ƽ������������0.3A���жϵ�ѹ���
            uint8_t i = 0;
            for(; i < 20; i++)
            {
                if(cap_vol_max_adjust.cap_v_queue[i] \
                   - cap_vol_max_adjust.cap_v_queue[(cap_vol_max_adjust.queue_tail + 1) % 20] \
                    > 0.1f)break;
            }
            if(i == 20)
            {
                //��������³��10����ѹ��������ΪIt/C=0.3*10/6.66=0.45V��i=20˵�����м�¼�ĵ�ѹ����
                //��û����0.1V�������ж�Ϊ����������©���������0.1A��
                float temp = 0;
                for(uint8_t j=0;j<20;j++)
                {
                    temp += cap_vol_max_adjust.cap_v_queue[j];
                }
                cap_vol_max_limit = temp / 20;
                cap_vol_max_adjust.time = 0;
            }
            else cap_vol_max_adjust.time = 8000;
        }
        
        cap_vol_max_adjust.queue_tail++;
        if(cap_vol_max_adjust.queue_tail > 19)cap_vol_max_adjust.queue_tail = 0;
    }
}

void pwm_output()
{
    //����ο����ϿƼ��ĳ���
    //����˼·�����ϡ�����STM32F334ͬ������BUCK-BOOST���ֵ�Դ��ơ�P8����ϸ˵��
    uint32_t	buck_duty,boost_duty;
	uint32_t	PWM_PER_0_5	= 0.5f * hhrtim1.Instance->sTimerxRegs[0].PERxR;
    
	if(pid_cap_i.OutDutyCycle > 0.9f)  //����ռ�ձȴ���90%ʱ������BOOSTģʽ��
		boost_duty = (pid_cap_i.OutDutyCycle - 0.8f) * PWM_PER_0_5;//����boost��ռ�ձȣ�����ģ�����ĶԳƵ�PWM��ע��Ҫ��ȥbuck_duty��ռ�ձ�: 0.8*PWM_PER_0_5
	else
		boost_duty = 0.1f * PWM_PER_0_5;	//����ռ�ձȲ�����90%ʱ������BUCKģʽ,boost_duty���̶�ռ�ձ�

	if(pid_cap_i.OutDutyCycle > 0.9f)
		buck_duty = 0.9f * PWM_PER_0_5; //����ռ�ձȴ���90%ʱ������BOOSTģʽ,buck_duty���̶�ռ�ձ�
	else
		buck_duty = pid_cap_i.OutDutyCycle * PWM_PER_0_5;	//����buck��ռ�ձȣ�����ģ�����ĶԳƵ�PWM��

	hhrtim1.Instance->sTimerxRegs[0].CMP1xR = PWM_PER_0_5 - buck_duty;//TimerA PWM��1�Ƚ�����ģ�����ĶԳƵ�PWM����PWM_PER_0_5Ϊ����
    hhrtim1.Instance->sTimerxRegs[0].CMP2xR = PWM_PER_0_5 + buck_duty;//TimerA PWM��0�Ƚ���������  ��400-50���루400+50�������ĵ�Ϊ400	

	hhrtim1.Instance->sTimerxRegs[3].CMP1xR = PWM_PER_0_5 + boost_duty;//TimerD PWM��1�Ƚ�����ģ�����ĶԳƵ�PWM����PWM_PER_0_5Ϊ����
    hhrtim1.Instance->sTimerxRegs[3].CMP2xR = PWM_PER_0_5 - boost_duty;//TimerD PWM��0�Ƚ���������  ��400-50���루400+50�������ĵ�Ϊ400	
    
}

void system_init()
{
    //KalmanCreate
    KalmanCreate(&p_bat_v,20,200);
    KalmanCreate(&p_bat_i,20,200);
    KalmanCreate(&p_cap_v,20,200);
    KalmanCreate(&p_cap_i,20,200);
    KalmanCreate(&p_chas_v,20,200);
    KalmanCreate(&p_chas_i,20,200);
    KalmanCreate(&p_bat_v_err,20,200);
    KalmanCreate(&p_bat_i_err,20,200);
    //opamp
    HAL_OPAMP_Start(&hopamp2);
    //hrtim
    HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_MASTER);
//    HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2|HRTIM_OUTPUT_TD1|HRTIM_OUTPUT_TD2); //ͨ����
    HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_A|HRTIM_TIMERID_TIMER_D); //�����Ӷ�ʱ��
    //adc
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_Delay(10);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_data.adc_list_record, 3);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    HAL_Delay(10);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)(&adc_data.adc_list_record[3]), 3);
    //dma�ж�
    hdma_adc1.XferCpltCallback = hdma_adc1_cplt;
    hdma_adc2.XferCpltCallback = hdma_adc2_cplt;
    //tim
    HAL_TIM_Base_Start_IT(&htim2);
    //can
    CAN_Init();
    //oled
    OLED_Init();
    //iwdg
//    HAL_IWDG_Init(&hiwdg);
    //flash
    Flash_Init();
}

void error_check()
{
    static uint32_t timetick[5] = {0};
    outtime[0] = (adc_data.cap_v > 25) ? ++outtime[0] : 0;  //���ݹ�ѹ��ʱ
    outtime[1] = (abs(adc_data.cap_i) > 14) ? ++outtime[1] : 0;  //���ݹ�����ʱ
    outtime[2] = (adc_data.cap_v < 3) ? ++outtime[2] : 0;  //����Ƿѹ��ʱ
    outtime[3] = (adc_data.bat_v < 16) ? ++outtime[3] : 0;  //����ϵͳǷѹ��ʱ
    outtime[4]++;
    
    if(outtime[0] > 10)
    {
        cap_state.bit.cap_v_over = 1;
    }
    else
    {
        cap_state.bit.cap_v_over = 0;
    }
    
    if(outtime[1] > 3)
    {
        cap_state.bit.cap_i_over = 1;
        timetick[1] = 0;
    }
    else if(cap_state.bit.cap_i_over == 1)
    {
        timetick[1]++;
        if(timetick[1] > 3000)
        {
            timetick[1] = 0;
            cap_state.bit.cap_i_over = 0;
        }
    }
    
    if(outtime[2] > 10)
    {
        cap_state.bit.cap_v_low = 1;
    }
    else
    {
        cap_state.bit.cap_v_low = 0;
    }
    
    if(outtime[3] > 10)
    {
        cap_state.bit.bat_v_low = 1;
        timetick[3] = 0;
    }
    else if(cap_state.bit.bat_v_low == 1)
    {
        timetick[3]++;
        if(timetick[3] > 3000)
        {
            timetick[3] = 0;
            cap_state.bit.bat_v_low = 0;
        }
    }
    
    if(outtime[4] > 1000)
    {
        cap_state.bit.can_receive_miss = 1;
        receive_data.cap_control.bit.cap_switch = 0;
    }
    else
    {
        cap_state.bit.can_receive_miss = 0;
    }
    
    if(cap_state.bit.cap_i_over == 1 || cap_state.bit.bat_v_low == 1 || (receive_data.cap_control.bit.cap_switch == 0 && adjust_switch == 0) || pwm_switch == 0)
    {
        HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2|HRTIM_OUTPUT_TD1|HRTIM_OUTPUT_TD2); //ͨ���ر�
        pid_cap_i.PidSwitch = 0;
        pid_bat_power.PidSwitch = 0;
        pid_powerbuffer.PidSwitch = 0;
    }
    else
    {
        pid_cap_i.PidSwitch = 1;
        pid_bat_power.PidSwitch = 1;
        pid_powerbuffer.PidSwitch = 1;
    }
}

void data_cal()
{
    //�����Ч��ֻ�ǿ��ư�����������Ч�ʣ������������������ĵ�Ч�ʡ�
    static uint16_t timetick = 0;
    static float powerin_start = 0;   //���չ���
    static float powerout_start = 0;  //��������
    
//    static float cap_i_rec[6] = {0};    //ÿ���¼һ�ε���
//    static float cap_v_rec[6] = {0};    //ÿ���¼һ�ε�ѹ
//    static float cap_i_integ[5] = {0};    //��¼��ε������֣�ÿ��ʱ��1s
//    static uint8_t counter = 0;     //������
    
    //����0�������ʣ�С��0���չ���
    bat_power = adc_data.bat_v * adc_data.bat_i;
    cap_power = -adc_data.cap_v * adc_data.cap_i;
    chas_power = -adc_data.chas_v * adc_data.chas_i;
    
    bat_power > 0 ? (powerout_rec += bat_power) : (powerin_rec += bat_power);
    cap_power > 0 ? (powerout_rec += cap_power) : (powerin_rec += cap_power);
    chas_power > 0 ? (powerout_rec += chas_power) : (powerin_rec += chas_power);
    
//    cap_i_integ[counter] += adc_data.cap_i;     //��������
//    if(counter == 0 && timetick == 0)
//    {
//        cap_i_rec[0] = adc_data.cap_i;
//        cap_v_rec[0] = adc_data.cap_v;    
//    }
    
    timetick ++;
    if(timetick > 999)
    {
        timetick = 0;
        //������һ���ڵ�Ч��
        efficiency_sec = -(powerin_rec - powerin_start) / (powerout_rec - powerout_start);
        powerin_start = powerin_rec;
        powerout_start = powerout_rec;
        //������ݺ�����
//        counter++;
//        cap_i_rec[counter] = adc_data.cap_i;
//        cap_v_rec[counter] = adc_data.cap_v;  
//        if(counter > 4)
//        {
//            //��С���˷�
//            float x[5] = {0};
//            float y[5] = {0};
//            float x_sum = 0;
//            float y_sum = 0;
//            float x2_sum = 0;
//            float xy_sum = 0;
//            counter = 0;
//            for(uint8_t i = 0; i < 5; i++)
//            {
//                x[i] = cap_i_integ[i] / (1000 * (cap_i_rec[i+1] - cap_i_rec[i]));
//                y[i] = (cap_v_rec[i+1] - cap_v_rec[i]) / (cap_i_rec[i+1] - cap_i_rec[i]);
//                x_sum += x[i];
//                y_sum += x[i];
//                x2_sum += x[i] * x[i];
//                xy_sum += x[i] * y[i];
//            }
//            capacity = (x2_sum - x_sum * x_sum / 5) / (xy_sum - y_sum * y_sum / 5);     //��λ��F
//            resistance = (y_sum - capacity * x_sum) * 200;    //��λ��m��
//        }
    }
}

void flash_loop()
{
    static uint32_t last_timetick = 0;
    if(HAL_GetTick() - last_timetick >= 1000 && receive_data.cap_control.bit.gamegoing == 1)
    {
        last_timetick = HAL_GetTick();
        flash_info.frame_header = 0xA5;
        flash_info.cap_control = receive_data.cap_control.all | ((cap_state.state >> 1) << 3);
        flash_info.chassis_power_buffer = receive_data.chassis_power_buffer;
        flash_info.chassis_volt = receive_data.chassis_volt;
        flash_info.chassis_current = receive_data.chassis_current;
        flash_info.output_power_limit = receive_data.output_power_limit;
        flash_info.cap_v = float_to_int16(adc_data.cap_v, 30, 0, 32000, -32000);
        Flash_Write();
    }
    Flash_Read();
    Flash_Erase();
    Flash_Empty_Check();
    Flash_usart_solve();
}

void cap_loop()
{
    static uint8_t timetick = 0;
    uint32_t load_tick = 0;
    load_tick = get_tickus();
    if(timetick++ >= 20)
    {
        timetick = 0;
        cap_pid_powerbuffer();
        can_send();
    }
//    HAL_IWDG_Refresh(&hiwdg);
    adc_solve();
    cap_pid_bat_power();
    data_cal();
    error_check();
    err_adjust();
    led_blue_set();
    led_red_set();
    key_scan();
    oled_cal();
    flash_loop();
    load.cap_loop = get_tickus() - load_tick;
    load.cap_loop_max = max(load.cap_loop_max, load.cap_loop);
}
void Startcap_loop(void *argument)
{
    TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();  //��ʼ��Ϊ����while֮ǰ�ĵδ����ֵ
    for(;;)
    {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));  //��ʱ
        cap_loop();
    }
}

void StartOLED(void *argument)
{
    for(;;)
    {
        oled_display();
        osDelay(200);
    }
}

void CAN_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{
    if(canId == 0x02E)
    {
        receive_data.chassis_power_buffer = ((uint16_t)rxBuf[0] << 8| rxBuf[1]);
//        receive_data.chassis_volt = ((uint16_t)rxBuf[2] << 8| rxBuf[3]);
//        receive_data.chassis_current = ((uint16_t)rxBuf[4] << 8| rxBuf[5]);
        if(receive_data.chassis_power_buffer > 60)receive_data.chassis_power_buffer = 60;
        if(receive_data.chassis_volt != ((uint16_t)rxBuf[2] << 8| rxBuf[3]))
        {
            receive_data.chassis_volt = ((uint16_t)rxBuf[2] << 8| rxBuf[3]);
            adc_data.bat_v_err =KalmanFilter(&p_bat_v_err, (float)receive_data.chassis_volt / 1000 - adc_data.bat_v_unmix);
        }
        if(receive_data.chassis_current != ((uint16_t)rxBuf[4] << 8| rxBuf[5]))
        {
            receive_data.chassis_current = ((uint16_t)rxBuf[4] << 8| rxBuf[5]);
            adc_data.bat_i_err =KalmanFilter(&p_bat_i_err, (float)receive_data.chassis_current / 1000 - adc_data.bat_i_unmix);
        }
    }
    else if(canId == 0x02F)
    {
        if(adjust_switch != 3)
        {
            receive_data.chassis_power_limit = ((uint16_t)rxBuf[0] << 8| rxBuf[1]);
            receive_data.output_power_limit = ((uint16_t)rxBuf[2] << 8| rxBuf[3]);
            receive_data.input_power_limit = ((uint16_t)rxBuf[4] << 8| rxBuf[5]);
            receive_data.cap_control.all = ((uint16_t)rxBuf[6] << 8| rxBuf[7]);
        }
        if(receive_data.chassis_power_limit > 120)receive_data.chassis_power_limit = 120;
        if(receive_data.output_power_limit > 300 || receive_data.output_power_limit < -120)
        {
            receive_data.output_power_limit = 300;
        }
        if(receive_data.input_power_limit > 150)receive_data.input_power_limit = 150;
    }
    outtime[4] = 0;
}

void can_send()
{
    can_send_data[0] = float_to_int16(adc_data.cap_v, 30, 0, 32000, -32000);
    can_send_data[1] = float_to_int16(adc_data.cap_i, 20, -20, 32000, -32000);
    can_send_data[2] = cap_state.state;
    CAN_SendData(0x030, can_send_data);
}

int16_t float_to_int16(float a, float a_max, float a_min, int16_t b_max, int16_t b_min)
{
    int16_t b = (a - a_min) / (a_max - a_min) * (float)(b_max - b_min) + (float)b_min + 0.5f;
    return b;
}

float int16_to_float(int16_t a, int16_t a_max, int16_t a_min, float b_max, float b_min)
{
    float b = (float)(a - a_min) / (float)(a_max - a_min) * (b_max - b_min) + b_min;
    return b;
}

void led_blue_set()
{
    if(HAL_GetTick() % 2000 > 1000)
    {
        LED_BLUE_ON();
    }
    else
    {
        LED_BLUE_OFF();
    }
}

void led_red_set()
{
    if(cap_state.bit.can_receive_miss == 1)
    {
        LED_RED_ON();
    }
    else
    {
        LED_RED_OFF();
    }
}

typedef struct
{
    float now;
    float max;
    float min;
    float aver;
    float start;
    float end;
    uint32_t count;
}f_value_cal_t;

f_value_cal_t cap_vol;
f_value_cal_t chas_pow;

typedef struct
{
    uint16_t now;
    uint16_t max;
    uint16_t min;
    float aver;
    uint16_t start;
    uint16_t end;
    uint32_t count;
}u8_value_cal_t;

u8_value_cal_t powerbuff;
float energy_rec = 0;
uint32_t time_rec = 0;

void oled_cal()
{
    static uint8_t record_state = 0;
    static float powerin_start = 0;
    static float powerout_start = 0;
    if(receive_data.cap_control.bit.cap_record == 1)
    {
        if(record_state == 0)
        {
            record_state = 1;
            
            cap_vol.now = adc_data.cap_v;
            cap_vol.max = cap_vol.now;
            cap_vol.min = cap_vol.now;
            cap_vol.aver = cap_vol.now;
            cap_vol.start = cap_vol.now;
            cap_vol.end = 0;
            cap_vol.count = 1;
            
            chas_pow.now = chas_power;
            chas_pow.max = chas_pow.now;
            chas_pow.min = chas_pow.now;
            chas_pow.aver = chas_pow.now;
            chas_pow.count = 1;
            
            powerbuff.now = receive_data.chassis_power_buffer;
            powerbuff.max = powerbuff.now;
            powerbuff.min = powerbuff.now;
            powerbuff.aver = powerbuff.now;
            powerbuff.start = powerbuff.now;
            powerbuff.end = 0;
            powerbuff.count = 1;
            
            powerin_start = powerin_rec;
            powerout_start = powerout_rec;
        }
        else
        {            
            cap_vol.now = adc_data.cap_v;
            cap_vol.max = (cap_vol.now > cap_vol.max) ? cap_vol.now : cap_vol.max;
            cap_vol.min = (cap_vol.now < cap_vol.min) ? cap_vol.now : cap_vol.min;
            cap_vol.aver = (cap_vol.aver * cap_vol.count + cap_vol.now) / (cap_vol.count + 1);
            cap_vol.count++;
            
            chas_pow.now = adc_data.chas_v * adc_data.chas_i;
            chas_pow.max = (chas_pow.now > chas_pow.max) ? chas_pow.now : chas_pow.max;
            chas_pow.min = (chas_pow.now < chas_pow.min) ? chas_pow.now : chas_pow.min;
            chas_pow.aver = (chas_pow.aver * chas_pow.count + chas_pow.now) / (chas_pow.count + 1);
            chas_pow.count++;
            
            powerbuff.now = receive_data.chassis_power_buffer;
            powerbuff.max = (powerbuff.now > powerbuff.max) ? powerbuff.now : powerbuff.max;
            powerbuff.min = (powerbuff.now < powerbuff.min) ? powerbuff.now : powerbuff.min;
            powerbuff.aver = (powerbuff.aver * powerbuff.count + powerbuff.now) / (powerbuff.count + 1);
            powerbuff.count++;
        }
    }
    else
    {
        cap_vol.now = adc_data.cap_v;
        chas_pow.now = adc_data.chas_v * adc_data.chas_i;
        powerbuff.now = receive_data.chassis_power_buffer;
        if(record_state == 1)
        {
            record_state = 0;
            cap_vol.end = cap_vol.now;
            chas_pow.end = chas_pow.now;
            powerbuff.end = powerbuff.now;
            efficiency_rec = -(powerin_rec - powerin_start) / (powerout_rec - powerout_start);
            time_rec = powerbuff.count - 2;
            energy_rec = chas_pow.aver * time_rec / 1000;
        }
    }
    
    if(key_press == 1)
    {
        key_press = 0;
        display_mode = !display_mode;
    }
}

void oled_display()
{
    char ch[4][22] = {0};
    static uint8_t last_mode = 1;
    uint32_t load_tick = 0;
    load_tick = get_tickus();
    if(display_mode == 1)
    {
        if(last_mode == 0)
        {
            last_mode = display_mode;
            OLED_Clear();
        }
        sprintf(ch[0], "U:%4.1f %4.1f %4.1f %4.1f", cap_vol.now, cap_vol.start, cap_vol.end, cap_vol.min);
        sprintf(ch[1], "P:%4.0f %4.0f %4.0f %4.0f", chas_pow.now, chas_pow.max, chas_pow.min, chas_pow.aver);
        sprintf(ch[2], "BF:%2d %2d %2d %2.0f %2d %2d", powerbuff.now, powerbuff.max, powerbuff.min, powerbuff.aver, powerbuff.start, powerbuff.end);
        sprintf(ch[3], "E:%-5.0f %5.3f T:%5.1f", energy_rec, efficiency_rec, (float)time_rec / 1000);

        OLED_ShowString(0, 0, (uint8_t *)ch[0], 12);
        OLED_ShowString(0, 1, (uint8_t *)ch[1], 12);
        OLED_ShowString(0, 2, (uint8_t *)ch[2], 12);
        OLED_ShowString(0, 3, (uint8_t *)ch[3], 12);
    }
    else
    {
        if(last_mode == 1)
        {
            last_mode = display_mode;
            OLED_Clear();
        }
        sprintf(ch[0], "BAT:%4.1f %5.1f %6.1f", adc_data.bat_v, adc_data.bat_i, bat_power);
        sprintf(ch[1], "CAP:%4.1f %5.1f %6.1f", adc_data.cap_v, adc_data.cap_i, cap_power);
        sprintf(ch[2], "CS:%4.1f %5.1f %6.1f", adc_data.chas_v, adc_data.chas_i, chas_power);
        sprintf(ch[3], "BF:%2d A:%-3.0f %4.1f %d", receive_data.chassis_power_buffer, power_adjust.val, cap_vol_max_limit, adc_data.num);

        OLED_ShowString(0, 0, (uint8_t *)ch[0], 12);
        OLED_ShowString(0, 1, (uint8_t *)ch[1], 12);
        OLED_ShowString(0, 2, (uint8_t *)ch[2], 12);
        OLED_ShowString(0, 3, (uint8_t *)ch[3], 12);
    }
    load.oled = get_tickus() - load_tick;
    load.oled_max = max(load.oled, load.oled_max);
}

uint32_t get_tickus()
{
    uint32_t ms, us;

	ms = HAL_GetTick();
	// ѡ�ö�ʱ��3��ΪHALʱ����TimeBase
	// Freq:1MHz => 1Tick = 1us
	// Period:1ms
	us = TIM3->CNT;
	
	return (ms*1000 + us);
}

void key_scan()
{
    static uint16_t key_rec = 0xffff;
    static uint8_t key_relax_flag = 1;
    key_rec = (key_rec << 1) | HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin);
    if(key_rec == 0 && key_relax_flag == 1)
    {
        key_press = 1;
        key_relax_flag = 0;
    }
    else if(key_rec == 0xff)
    {
        key_relax_flag = 1;
    }
}


