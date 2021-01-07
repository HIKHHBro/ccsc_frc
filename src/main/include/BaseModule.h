/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#pragma once
#include <iostream>

typedef struct Gyro
{
    float acc_x = 0;
    float acc_y = 0;
    float acc_z = 0;

    float v_x = 0;
    float v_y = 0;
    float v_z = 0;

    float yaw = 0;
    float pitch = 0;
    float roll = 0;

}Gyro;

class RampFunction
{
public:
    RampFunction(float k){this->k = k;};
    ~RampFunction();
    float set(float value)
    {
        if(value - last_value >k)
        {
            value = k + last_value;
        }
        if(value - last_value <-k)
        {
            value = last_value - k;
        }
        last_value = value;
        return value;
    }
    void set_k(float k){this->k = k;};
private:
    float k;
    float last_value;
};
#define IS_SECTION(data_,min,max) (  ((data_) > (min) && (data_) < (max)) ?true:fasle ) 
#define  DEG_TO_RAD(deg_) ((deg_) /(45.0 / atan(1.0)) )
#define RAD_TO_DEG(rag_) ((rag_) *(45.0 / atan(1.0)) )
///< x 为减速前,y为减速后
inline  double reduction_ratio(double x,double y) { return (y/x);}


/* 设置模式 */
//比赛的时候一定要注释掉调试模式
#define GIMBAL_DEBUG //设置云台调试模式
// #define CHASSIS_DEBUG //设置底盘调试模式
#define COM_DEBUG //在公司调试
#define DIALS_DEBUG