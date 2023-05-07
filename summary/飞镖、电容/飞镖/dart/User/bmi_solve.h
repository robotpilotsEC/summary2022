#ifndef __BMI_SOLVE_H
#define __BMI_SOLVE_H

#define BMI_PT 0
#define BMI_CHAS 1

typedef struct {
    short ggx,ggy,ggz,aax,aay,aaz;
    float pitch,roll,yaw;
    float yaw_sum;
    char init_flag;
    float yaw_prev;
}bmi_info_t;

extern bmi_info_t bmi_info[];

void IMU_yaw_sum(bmi_info_t *bmi_info);


#endif


