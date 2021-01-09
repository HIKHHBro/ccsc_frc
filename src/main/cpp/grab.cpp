#include "grab.h"
#ifdef GRAB_DEBUG
#include <frc/smartdashboard/smartdashboard.h>
#endif
Grab::Grab(int pwm_c,int id,int pcm_c,float simpe_time)
{
    pwm_channel = pwm_c;
    can_id = id;
    pcm_channel = pcm_c;
    simpe_time = simpe_time;
    motor = new rev::SparkMax(pwm_channel);
    solenoid[0] = new frc::Solenoid(can_id,pcm_channel);
    solenoid[1] = new frc::Solenoid(can_id,pcm_channel+1);
    ramp_func = new RampFunction(0.01);
}

Grab::~Grab()
{
}
///< 放下抓取
bool Grab::put_down()
{
    solenoid[0]->Set(true);
    solenoid[1]->Set(true);
    motor->Set(ramp_func->set(speed));
    // if(cal_run_count())
        is_put_down = true;
    return cal_run_count();
}
///< 抬起
bool Grab::put_up()
{
    solenoid[0]->Set(true);
    solenoid[1]->Set(false);
    motor->Set(ramp_func->set(0));
    is_put_down = false;
    return cal_run_count();
}
///< 运行时间记数
bool Grab::cal_run_count()
{
    if( run_count < (1/simpe_time) && is_put_down == true)
    {
        run_count++;
        return false;
    }
    else 
    {
        run_count = 0;
        return true;
    }

}

///< 设置抓取传送速度  -1 ~1;
void Grab::set_speed(float s)
{
    speed = limit(s,-1.0,1.0);
}
#ifdef GRAB_DEBUG
void Grab::display()
{
    // frc::SmartDashboard::PutNumber("is_put_down:", is_put_down);
    // frc::SmartDashboard::PutNumber("motor_pwm:", ramp_func->get_last_data());
    // frc::SmartDashboard::PutNumber("set_max_pwm:", speed);
    frc::SmartDashboard::PutNumber("set_max_pwm", speed);

}
void Grab::debug()
{
    // display
    double s  = frc::SmartDashboard::GetNumber("set_max_pwm",0);
    if((s != speed)) { speed = s;}
    std::cout<<speed<<std::endl;
    frc::SmartDashboard::PutNumber("is_put_down:", is_put_down);
    frc::SmartDashboard::PutNumber("motor_pwm:", ramp_func->get_last_data());
    

}
#endif
