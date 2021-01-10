#include "grab.h"
#ifdef GRAB_DEBUG
#include <frc/smartdashboard/smartdashboard.h>
#endif
Grab::Grab(int pwm_c,int id,int pcm_c,float simpe_time,float acc):Neo(pwm_c,acc)
{
    can_id = id;
    pcm_channel = pcm_c;
    simpe_time = simpe_time;
    solenoid[0] = new frc::Solenoid(can_id,pcm_channel);
    solenoid[1] = new frc::Solenoid(can_id,pcm_channel+1);
}

Grab::~Grab()
{
    delete solenoid[0];
    delete solenoid[1];
}
//TODO: 使用延迟 待增加反馈
///< 放下抓取
bool Grab::put_down()
{
    if(is_put_down == false && !cal_run_count())
    {
        solenoid[0]->Set(true);
        solenoid[1]->Set(true);
        std::cout<<"dow"<<std::endl;
    }
    else
    {
        is_put_down = true;
        loosen_gas();
        std::cout<<"loss"<<std::endl;
    }    
    Set(cal_speed(speed));
    return is_put_down;
}
///< 抬起
bool Grab::put_up()
{
    solenoid[0]->Set(true);
    solenoid[1]->Set(false);
    Set(cal_speed(0));
    is_put_down = false;
    runned_count = 0;
    return is_put_down;
}
//TODO: 编写松开气缸和何时松开气缸
///< 松开气缸
bool Grab::loosen_gas()
{
    solenoid[0]->Set(false);
    solenoid[1]->Set(false);
    return true;
}
///< 运行时间记数
bool Grab::cal_run_count()
{
    if(is_put_down == false)
    {
        if( runned_count < runned_time*(1.0/simpe_time))
        {
            runned_count++;
            return false;
        }
    }
    runned_count =0;
    return true;
}

///< 设置抓取传送速度  -1 ~1;
void Grab::set_speed(float s)
{
    speed = limit(s,-1.0,1.0);
}
///< 设置抓取传送加速度  -0 ~ 1;
void Grab::set_acc(float a)
{
    set_k(limit(a,0.0,1.0));
}
#ifdef GRAB_DEBUG
void Grab::display()
{
    frc::SmartDashboard::PutNumber("set_max_pwm",speed);
    frc::SmartDashboard::PutNumber("set_acc",get_k());
    frc::SmartDashboard::PutNumber("runned_time",runned_time);
}
void Grab::debug()
{
    
    double s  = frc::SmartDashboard::GetNumber("set_max_pwm",speed);
    if((s != speed)) { set_speed(s);}

    double a  = frc::SmartDashboard::GetNumber("set_acc",get_k());
    if((a != get_k())) { set_acc(a);}

    float rt  = frc::SmartDashboard::GetNumber("runned_time",runned_time);
    if((rt != runned_time)) { runned_time = limit(rt,0.02,5.0);}

    // display
    frc::SmartDashboard::PutNumber("is_put_down", is_put_down);
    frc::SmartDashboard::PutNumber("motor_pwm", get_last_data());
    frc::SmartDashboard::PutNumber("runned_count",runned_count);
    frc::SmartDashboard::PutNumber("runned_count_max",runned_time*(1.0/simpe_time));
    frc::SmartDashboard::PutNumber("max_rpm",max_rpm);

}
#endif
