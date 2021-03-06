/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#pragma once
#include "BaseModule.h"
#include "AHRS.h"
#include "ctre/Phoenix.h"
#include <frc/controller/PIDController.h>
using namespace frc;
class Chassis 
{
 public:
    Chassis();
    ~Chassis();
    enum MotionModel {
        OMNIDIR4 =2, 
        STEER = 4};
    enum MOTOR {M1,M2,M3,M4,M_ALL};
    enum TARGET {x,y,z};
    enum Reference{WORLD,CAR};
    float speed[4];
    float wheel_s[4];
    float milemeter[3];
    double motion[3];
    void motion_model(float vx,float vy,float vz);
    float theta_m;//TODO: 
    bool check_gyro();
    void get_gyro();
    void get_run_target(float* target);
    void update_rc_data();
    void rc_run(float vx,float vy,float vz);
    bool auto_run(void);
    bool milemter();
    float series_to_mm (int16_t  wheel);
    bool to_position(float x,float y,float w);
    float wheel_theta = 45;
    float chassis_r = 382.835;//mm
    float wheel_r =  76.2;//mm
    float  motor_series = 2048;
    TalonFX* motor[4];
    float rc_control_x_y = 0.4;//控制x和y最大速度系数
    float rc_control_w = 0.1;//控制旋转最大速度系数
    int wheel_rc_to_sensor(float);
    void set_reference(Reference ref);
    float w_rc_to_sensor(float value);
    void updata_series(void);//更新和累计编码器值
    void set_series(int);//重设编码器值
    long int series_position[M_ALL];
 private:
    float angle_to_radian = 0.01745f;//锟角讹拷转锟斤拷锟斤拷
    float auto_angle = 45;
    AHRS *ahrs;
    RampFunction* ramp_func;
    Reference reference = CAR;//默认机体坐标系
    frc2::PIDController *auto_run_map_pid[3];



public:
    const int map_len = 2;
    const double map[2][3] = 
    {
        {0,0,10},
        {1,1,10}
    };


};






