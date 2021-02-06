#ifndef __SHOOT_H
#define __SHOOT_H
#include "BaseModule.h"
#include "ctre/Phoenix.h"
#include "my_thread.h"
#include <frc/DigitalInput.h>
#ifdef SHOOT_DEBUG
#include <frc/smartdashboard/smartdashboard.h>
#endif
class Shoot : public Falcon, public MyThread
{
public:
    enum MOTOR{Hor_tr,Hor_tr_u,Ver_tr,Sh1,Sh2,ALL};
    Shoot(int pwm_c,int can_id);
    ~Shoot();
    bool open_vertical_transfer();
    void open_horizontal_transfer();
    void close_vertical_transfer();
    void close_horizontal_transfer();
    void carry_out(MOTOR M,float rpm);
    void set_gimbal_angle(float angle);
    void set_gimbal_percent(float percent);
    bool reset();
    void start_shoot();
    void stop_shoot();
    bool stop_vertical_transfer();
    TalonFX* gimbal_motor;

#ifdef SHOOT_DEBUG
    void display() override;
    void debug()   override;
#endif
private:
    void run()   override;

    frc::DigitalInput* reset_sw;
    Neo* motor[ALL];
    float acc[ALL] = {0.01,0.01,0.05,0.1,0.1};
    float smoothing = 0;
    float kp = 0.1;
    float kf = 0.2;
    float max_angle = 30;//云台最大转动角度
    float reset_speed = 100;//rpm 0~6000
    float reset_acc = 0;//rpm
    float reset_output = 0.1;//0~1
    float reset_current_thres = 10;//amps
    float reset_speed_thres = 30;//100ms转的刻度
    float reset_period = 10000;//us
    bool is_reseted = false;
    int reset_error_count = 0;
    int reset_error_thre =  (int)(1000000.0/reset_period * 0.3);
/*
 * 电机方向:
 * Hor_tr: 顺时针 负
 * Ver_tr: 逆时针 正
 *    Sh1: 逆时针
 *    Sh2: 逆时针
 */
    float neo_speed[ALL] = {0.20,0.20,-0.25,-0.7,-0.7};//RPM 最大5700
};





#endif