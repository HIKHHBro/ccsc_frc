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

class Grab :public Neo
{
private:
    frc::Solenoid *solenoid[2];
    frc::Compressor *compressor;
    int pcm_channel = 1;
    int can_id = 20;
    bool is_put_down = false;
    int runned_count = 0; 
    float runned_time = 0.5;  //一秒动作执行完毕
    float simpe_time =0.02;
    float speed = 0.3; //-1 ~ 1
    // float acc = 0.01;// 每simpe_time时间增加0.01个单位速度 simpe_time = 0.02s
public:
    Grab(int pwm_c, int id, int pcm_c,float simpe_time ,float acc);
    ~Grab();
    bool put_down();
    bool put_up();
    bool cal_run_count();
    void set_speed(float);
    void set_acc(float a);
    bool loosen_gas();
    void enable_compressor();
#ifdef GRAB_DEBUG
    void display() override;
    void debug() override;
#endif

};


#endif