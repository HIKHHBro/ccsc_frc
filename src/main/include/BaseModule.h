/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#pragma once
#include <iostream>
#include "rev/SparkMax.h"
#include "ctre/Phoenix.h"

#define IS_SECTION(data_,min,max) (  ((data_) > (min) && (data_) < (max)) ?true:false) 
#define IS_X_SECTION(data_,x)   (((data_) > (-abs(x)) && (data_) < (abs(x))) ?true:false) 

#define LIMIT(data_,min,max) { (data_) = ((data_) < (min) ? (min) : (data_)); (data_) = ((data_) > (max) ? (max) : (data_));}
#define  DEG_TO_RAD(deg_) ((deg_) /(45.0 / atan(1.0)) )
#define RAD_TO_DEG(rag_) ((rag_) *(45.0 / atan(1.0)) )
///< x 为减速前,y为减速后
inline  double reduction_ratio(double x,double y) { return (y/x);}
class Base
{
public:
     Base(){};
     ~Base(){};
    inline int limit(int data,int min,int max)
    {data = (data) > (max) ? (max) : (data);
     data = (data) < (min) ? (min) : (data);
     return  data;} 
    inline float limit(float data,float min,float max)
    {data = (data) > (max) ? (max) : (data); 
    data = (data) < (min) ? (min) : (data);
    return  data;}
    virtual void display();
    virtual void debug();
};
class RampFunction
{
public:
    
    RampFunction(float k);
    ~RampFunction();
    float cal_speed(float value)
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
    };
    inline float get_last_data(){return last_value;};
    void set_k(float k){this->k = k;};
    float get_k(){return this->k;};
    float is_complete_acc(float speed){return IS_X_SECTION((speed - last_value),k); }//TODO: 待测试
private:
    float k;
    float last_value;
};
class Motor : public Base
{
private:
    /* data */
public:
    Motor(/* args */);
    Motor(double rpm,int enc,float redu);
    ~Motor();
    double max_rpm = 100;//转/s
    int encoder_l = 2048;//一圈电机编码器位数
    float reduction_ratiop = 1;
    void set_reduction_ratiop(float);
    void set_reduction_ratiop(int,int);
    void set_reduction_ratiop(float,float);
    void set_reduction_ratiop(int,int,int);
    void set_reduction_ratiop(int,int,int,int);
    double per_to_rpm(float);
    float rpm_to_per(float);
    float angle_to_enc(float input){return (input/360.0/reduction_ratiop); }//TODO: 待测试
    float enc_to_angle(int enc){return ((float)enc * 360.0 * reduction_ratiop);}//TODO: 待测试
};

class Falcon : public Motor
{
public:
    Falcon();
    ~Falcon();
    float enc_100ms_to_rpm(float);
    int rpm_to_enc_100ms(float);
    int mm_to_enc(float);
    double enc_to_mm(int);
    void set_dia(float);
    float d = 100;//mm
    int get_position_error(int target,int real);
    int mms_to_enc100ms(float mms);
    float enc100ms_to_mms(int enc);
};
class Neo : public Motor,public rev::SparkMax,public RampFunction
{
public:
    Neo(int channel,float k);
    ~Neo();
};

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




/* 设置模式 */
//比赛的时候一定要注释掉调试模式
// #define GIMBAL_DEBUG //设置云台调试模式
#define CHASSIS_DEBUG //设置底盘调试模式
// #define COM_DEBUG //在公司调试
// #define DIALS_DEBUG
// #define JOY_RC
#define XBON_RC
#define RC_DEBGU
// #define GRAB_DEBUG
// #define LIFT_DEBUG
// #define SHOOT_DEBUG
// #define SHOOT_DEBUG

