#ifndef __SHOOT_H
#define __SHOOT_H
#include "BaseModule.h"
#include "ctre/Phoenix.h"
#include "my_thread.h"
#ifdef SHOOT_DEBUG
#include <frc/smartdashboard/smartdashboard.h>
#endif
class Shoot : public Falcon, public MyThread
{
public:
    enum MOTOR{Hor_tr,Ver_tr,Sh1,Sh2,ALL};
    Shoot(int pwm_c,int can_id);
    ~Shoot();
    bool open_vertical_transfer();
    void open_horizontal_transfer();
    void close_vertical_transfer();
    void close_horizontal_transfer();
    void carry_out(MOTOR M,float rpm);
    void set_gimbal_angle(float angle);
    bool reset();

    TalonFX* gimbal_motor;

#ifdef SHOOT_DEBUG
    void display() override;
    void debug()   override;
#endif
private:
    void run()   override;

    
    Neo* motor[ALL];
    float acc[ALL] = {0.01,0.01,0.01,0.01};
    float smoothing = 0;
    float kp = 0.1;
    float kf = 0.2;
    float max_angle = 40;//云台最大转动角度
    float reset_speed = 100;//rpm 0~6000
    float reset_acc = 0;//rpm
    float reset_output = 0.1;//0~1
    float reset_current_thres = 10;//amps
    bool is_reseted;
/*
 * 电机方向:
 * Hor_tr: 顺时针 负
 * Ver_tr: 逆时针 正
 *    Sh1: 逆时针
 *    Sh2: 逆时针
 */
    float neo_speed[ALL] = {0,0,0,0};//RPM 最大5700
};





#endif