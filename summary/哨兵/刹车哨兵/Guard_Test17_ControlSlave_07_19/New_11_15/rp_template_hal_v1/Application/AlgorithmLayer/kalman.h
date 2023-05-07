#ifndef _KALMAN_H
#define _KALMAN_H

#include "rp_config.h"
#include "arm_math.h"

#define mat         arm_matrix_instance_f32 //float
#define mat_64      arm_matrix_instance_f64 //double
#define mat_init    arm_mat_init_f32
#define mat_add     arm_mat_add_f32
#define mat_sub     arm_mat_sub_f32
#define mat_mult    arm_mat_mult_f32
#define mat_trans   arm_mat_trans_f32//�������ת��
#define mat_inv     arm_mat_inverse_f32
#define mat_inv_f64 arm_mat_inverse_f64

typedef struct
{
  float raw_value;
  float filtered_value[2];
  mat xhat, xhatminus, z, A, H, AT, HT, Q, R, P, Pminus, K;
} kalman_filter_t;

typedef struct
{
  float raw_value;
  float filtered_value[2];
  float xhat_data[2], xhatminus_data[2], z_data[2],Pminus_data[4], K_data[4];
  float P_data[4];
  float AT_data[4], HT_data[4];
  float A_data[4];
  float H_data[4];
  float Q_data[4];
  float R_data[4];
} kalman_filter_init_t;

typedef struct {
    float X_last; //��һʱ�̵����Ž��  X(k-|k-1)
    float X_mid;  //��ǰʱ�̵�Ԥ����  X(k|k-1)
    float X_now;  //��ǰʱ�̵����Ž��  X(k|k)
    float P_mid;  //��ǰʱ��Ԥ������Э����  P(k|k-1)
    float P_now;  //��ǰʱ�����Ž����Э����  P(k|k)
    float P_last; //��һʱ�����Ž����Э����  P(k-1|k-1)
    float kg;     //kalman����
    float A;      //ϵͳ����
	float B;
    float Q;
    float R;
    float H;
}extKalman_t;

extern extKalman_t rc_gimbal_ch0,rc_gimbal_ch1;
extern extKalman_t imu_roll_rateKF,imu_real_yaw_rateKF;
/* vision */
extern extKalman_t vision_distance_get;
extern extKalman_t vision_pitch_raw_filtered,vision_yaw_raw_filtered;

extern extKalman_t pitch_measure_rateKF,yaw_measure_rateKF;
extern extKalman_t pitch_real_target_KF,yaw_real_target_KF;

void Kalman_Init(void);
void   kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I);
float *kalman_filter_calc(kalman_filter_t *F, float signal1, float signal2);
void   KalmanCreate(extKalman_t *p,float T_Q,float T_R);
float  KalmanFilter(extKalman_t* p,float dat);

#endif
