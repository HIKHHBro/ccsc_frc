#include "lifting.h"
Lifting::Lifting(int id)
{
    set_reduction_ratiop(1,100);//1为位移输入,100为电机输出
    set_dia(50);//直径50mm
    for(int i =0;i<M_ALL;i++)
    {
        motor[i] = new TalonFX(i+id);  

        /* Factory default hardware to prevent unexpected behavior */
        motor[i]->ConfigFactoryDefault();

        /* Configure Sensor Source for Pirmary PID */
        motor[i]->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor,0, 10);

        motor[i]->SetInverted(TalonFXInvertType::CounterClockwise);

        /* Set relevant frame periods to be at least as fast as periodic rate */
        motor[i]->SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
        motor[i]->SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);

        /* Set the peak and nominal outputs */
        motor[i]->ConfigNominalOutputForward(0, 10);
        motor[i]->ConfigNominalOutputReverse(0, 10);
        motor[i]->ConfigPeakOutputForward(1, 10);
        motor[i]->ConfigPeakOutputReverse(-1, 10);

        /* Set Motion Magic gains in slot0 - see documentation */
        motor[i]->SelectProfileSlot(0, 0);
        motor[i]->Config_kF(0, 0.3, 10);
        motor[i]->Config_kP(0, 0.1, 10);
        motor[i]->Config_kI(0, 0.0, 10);
        motor[i]->Config_kD(0, 0.0, 10);

        /* Set acceleration and vcruise velocity - see documentation */
        motor[i]->ConfigMotionCruiseVelocity(1500, 10);
        motor[i]->ConfigMotionAcceleration(1500, 10);

        /* Zero the sensor */
        motor[i]->SetSelectedSensorPosition(0, 0, 10);
        motor[i]->ConfigMotionSCurveStrength(smoothing, 0);
    }
    motor[0]->Follow(*motor[1],FollowerType::FollowerType_AuxOutput1);
}           


Lifting::~Lifting()
{
}
///< 设置电机执行
void Lifting::set_point(float len)
{
    motor[0]->Set(ControlMode::MotionMagic, mm_to_enc(len) + len_comp);
    motor[1]->Set(ControlMode::MotionMagic, mm_to_enc(len) + len_comp);
}
///< 提升
//TODO: 测试获取误差的api是否是GetClosedLoopError
bool Lifting::lift()
{
    if(carry_out(lift_high,stretch_speed))
    {
        is_stretched = true;
        return true;
    }
    return false;
}
//TODO: 待测试
///< 伸出
bool Lifting::stretch_out()
{
    if(carry_out(route,stretch_speed))
    {
        is_stretched = true;
    }
    return false;
}
///< 动作执行 s 位移 v 速度
bool Lifting::carry_out(float s,float v)
{
    float error[2],v_temp[2];
    motor[0]->ConfigMotionCruiseVelocity(v, 10);
    motor[1]->ConfigMotionCruiseVelocity(v, 10);
    for(int i = 0;i<M_ALL;i++)
    {
        error[i] = motor[i]->GetClosedLoopError();
        v_temp[i] = motor[i]->GetSelectedSensorVelocity();
    }
    if(IS_SECTION(error[0],-pos_thres,pos_thres) && v_temp[1] == speed_thres)
    {
        return true;
    }
    else
    {
        set_point(s);
        return false;
    }
}
///< 收缩
bool Lifting::shrink()
{
    if(carry_out(0,stretch_speed))
    {
        is_stretched = false;
    }
    return false;
}
//TODO：线程函数
///< 复位
bool Lifting::reset()
{
    float error[2],v[2];
    motor[0]->ConfigClosedLoopPeakOutput(0,reset_output,0);
    motor[1]->ConfigClosedLoopPeakOutput(0,reset_output,0);
    for(int i = 0;i<M_ALL;i++)
    {
        if(motor[i]->GetOutputCurrent() > reset_current_thres && motor[0]->GetSelectedSensorVelocity()==0)
        {
            if(get_reset_key(M1))
            {
                motor[i]->SetSelectedSensorPosition(0, 0, 10);
                continue;
            }
        }
        motor[i]->Set(ControlMode::Velocity,reset_speed);

    }
    return true;
}
//TODO: 待写按键输入
///< 获取复位按键值
bool Lifting::get_reset_key(MOTOR M)
{
    return false;
}
///< 是否完全伸长
bool Lifting::is_stretch_out()
{
    return is_stretched;
}
#ifdef LIFT_DEBUG
void Lifting::display()
{
//  frc::SmartDashboard::PutNumber("enc_get",0);
 frc::SmartDashboard::PutNumber("mm_get",0);
 frc::SmartDashboard::PutNumber("enc_get",0);

}
void Lifting::debug()
{

// int enc = frc::SmartDashboard::GetNumber("enc_get",0);
// int per = frc::SmartDashboard::GetNumber("per_get",0);
// int rpm = frc::SmartDashboard::GetNumber("rpm_get",0);
// display
//  frc::SmartDashboard::PutNumber("enc_show",rpm_to_enc(rpm));
//  frc::SmartDashboard::PutNumber("per",rpm_to_per(rpm));
//  frc::SmartDashboard::PutNumber("rpm",enc_to_rpm(enc));
//  frc::SmartDashboard::PutNumber("max_rpm",max_rpm);
//  frc::SmartDashboard::PutNumber("perrpm_show",per_to_rpm(per));
//  frc::SmartDashboard::PutNumber("enc_how",rpm_to_enc(per_to_rpm(per)));


int mm_get = frc::SmartDashboard::GetNumber("mm_get",0);
int enc_get = frc::SmartDashboard::GetNumber("enc_get",0);
 frc::SmartDashboard::PutNumber("mm",enc_to_mm(enc_get));
 frc::SmartDashboard::PutNumber("enc",mm_to_enc(mm_get));

//  frc::SmartDashboard::PutNumber("position_to_enc_coep",position_to_enc_coep);
 frc::SmartDashboard::PutNumber("reduction_ratiop",reduction_ratiop);
 frc::SmartDashboard::PutNumber("d",d);

}
#endif