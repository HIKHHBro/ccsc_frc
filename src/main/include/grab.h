#ifndef __GRAB_H
#define __GRAB_H
#include "BaseModule.h"
#include "rev/SparkMax.h"
#include "ctre/Phoenix.h"
#include <frc/Compressor.h>
#include <frc/Solenoid.h>
#ifdef GRAB_DEBUG
#include <frc/smartdashboard/smartdashboard.h>
#endif
//TODO: 转速单位转换 加速度单位转换
class Grab :public Base
{
private:
    rev::SparkMax *motor;
    frc::Solenoid *solenoid[2];
    RampFunction* ramp_func;
    int pcm_channel = 1;
    int pwm_channel = 0;
    int can_id = 20;
    bool is_put_down = false;
    int run_count = 0;  //一秒动作执行完毕
    int simpe_time =0.02;
    float speed = 0.1; //-1 ~ 1
    float acc = 0.01;// 每simpe_time时间增加0.01个单位速度 simpe_time = 0.02s
public:
    Grab(int pwm_c, int id, int pcm_c,float simpe_time);
    ~Grab();
    bool put_down();
    bool put_up();
    bool cal_run_count();
    void set_speed(float);

#ifdef GRAB_DEBUG
    void display();
    void debug();
#endif
};


#endif