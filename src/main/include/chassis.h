/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#ifndef __CHASSIS_H
#define __CHASSIS_H
#include "BaseModule.h"
#include "AHRS.h"
#include "ctre/Phoenix.h"
#include <frc/controller/PIDController.h>
#include "pid_control.h"
#include <signal.h>
#include "my_thread.h"
#include <atomic>
#include <thread>
#include <unistd.h>
#ifdef CHASSIS_DEBUG
#include <frc/smartdashboard/smartdashboard.h>
#endif
using namespace frc;
class Chassis :public Falcon,public MyThread
{
 public:
    Chassis(int can_id);
    ~Chassis();
    enum MotionModel {
        OMNIDIR4 =2, 
        STEER = 4};
    enum MOTOR {M1,M2,M3,M4,M_ALL};
    enum TARGET {x,y,z,all_dir};
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
    bool milemter();
    float series_to_mm (int16_t  wheel);
    bool to_position(float x,float y,float w);
    float wheel_theta = 45;
    float chassis_r = 382.835;//mm//382.835
    TalonFX* motor[4];
    float rc_control_x_y = 1;//控制x和y最大速度系数
    float rc_control_w = 0.2;//控制旋转最大速度系数 //TODO: 精度不够
    int wheel_rc_to_sensor(float);
    void set_reference(Reference ref);
    float w_rc_to_sensor(float value);
    void updata_series(void);//更新和累计编码器值
    void set_series();//重设编码器值
    void start_auto_run(void);
    bool get_auto_run_status();
    void exit_auto_run();
    void motor_init(int id);
    bool get_auto_run_is_finished();
    void chassis_pid_loop();
#ifdef CHASSIS_DEBUG
    void debug() override;
    void display() override;
#endif
 private:
    float angle_to_radian = 0.01745f;
    float auto_angle = 45;
    float world_angle;
    AHRS *ahrs;
    Reference reference = CAR;//默认机体坐标系
    frc2::PIDController *auto_run_map_pid[3];
    void auto_run();
    bool auto_run_is_finished =false;
    float target_vel[all_dir];
    void run() override;
    PIDControl *motor_pid[4];
    void pid_loop();
    std::thread pid_thread;
    std::atomic<bool> is_interript_pid = false;
    float speed_loop_kp = 0.05;
    float speed_loop_ki = 0.05;

    float pos_loop_kp = 0.05;
    float pos_loop_ki = 0;
    float pos_loop_kd = 0;
    float is_arrived_pos_error[2] = {10,10};
    float is_arrived_vel_error[2] = {10,10};

   //调试变量，方便查看
   float y_pos_error= 0;
   float x_pos_error= 0;
   float x_v_error= 0;
   float y_v_error= 0;
   float auto_output[3];
public:
    const int map_len = 2;
    
    float map[2][3] = 
    {
      /* {x(mm),y(mm),speed(mms)}*/
        {0,0,10},
        {1,4000,10}
    };

};

#endif

