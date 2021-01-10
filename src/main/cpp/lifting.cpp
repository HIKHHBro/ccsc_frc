#include "lifting.h"
Lifting::Lifting(int id)
{
    set_reduction_ratiop(1,100);//1为位移输入,100为电机输出
    set_dia(50);//直径50mm
    set_loop_time(50000);//50ms
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
    //TODO: 待测试跟随
    // motor[0]->Follow(*motor[1],FollowerType::FollowerType_AuxOutput1);
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
void Lifting::run()
{

    motor[0]->ConfigClosedLoopPeakOutput(0,reset_output,0);
    motor[1]->ConfigClosedLoopPeakOutput(0,reset_output,0);
    while (!isInterrupted())
    {
        std::cout<<"初始化"<<std::endl;
        thread_sleep();
    }
    
    // for(int i = 0;i<M_ALL;i++)
    // {
    //     if(motor[i]->GetOutputCurrent() > reset_current_thres && motor[0]->GetSelectedSensorVelocity()==0)
    //     {
    //         if(get_reset_key(M1))
    //         {
    //             motor[i]->SetSelectedSensorPosition(0, 0, 10);
    //             continue;
    //         }
    //     }
    //     motor[i]->Set(ControlMode::Velocity,reset_speed);

    // }


    

}
#ifdef LIFT_DEBUG
void Lifting::display()
{
 frc::SmartDashboard::PutNumber("smoothing",smoothing);
 frc::SmartDashboard::PutNumber("len_comp",len_comp);
 frc::SmartDashboard::PutNumber("route",route);
 frc::SmartDashboard::PutNumber("lift_high",lift_high);
 frc::SmartDashboard::PutNumber("pos_thres",pos_thres);
 frc::SmartDashboard::PutNumber("speed_thres",speed_thres);
 frc::SmartDashboard::PutNumber("stretch_speed",stretch_speed);
 frc::SmartDashboard::PutNumber("is_stretched",is_stretched);
 frc::SmartDashboard::PutNumber("reset_speed",reset_speed);
 frc::SmartDashboard::PutNumber("reset_output",reset_output);
 frc::SmartDashboard::PutNumber("reset_current_thres",reset_current_thres);
}
void Lifting::debug()
{

    int get1 = frc::SmartDashboard::GetNumber("smoothing",smoothing);
    if(get1 != smoothing) {smoothing = get1;}
    int get2 = frc::SmartDashboard::GetNumber("len_comp",len_comp);
    if(get2 != len_comp) {len_comp = get2;}
    int get3 = frc::SmartDashboard::GetNumber("route",route);
    if(get3 != route) {route = get3;}
    int get4 = frc::SmartDashboard::GetNumber("lift_high",lift_high);
    if(get4 != lift_high) {lift_high = get4;}
    int get5 = frc::SmartDashboard::GetNumber("pos_thres",pos_thres);
    if(get5 != pos_thres) {pos_thres = get5;}
    int get6 = frc::SmartDashboard::GetNumber("speed_thres",speed_thres);
    if(get6 != speed_thres) {speed_thres = get6;}
    int get7 = frc::SmartDashboard::GetNumber("stretch_speed",stretch_speed);
    if(get7 != stretch_speed) {stretch_speed = get7;}
    int get8 = frc::SmartDashboard::GetNumber("is_stretched",is_stretched);
    if(get8 != is_stretched) {is_stretched = get8;}
    int get9 = frc::SmartDashboard::GetNumber("reset_speed",reset_speed);
    if(get9 != reset_speed) {reset_speed = get9;}
    int get10 = frc::SmartDashboard::GetNumber("reset_output",reset_output);
    if(get10 != reset_output) {reset_output = get10;}
    int get11 = frc::SmartDashboard::GetNumber("reset_current_thres",reset_current_thres);
    if(get11 != reset_current_thres) {reset_current_thres = get11;}

    frc::SmartDashboard::PutNumber("motor_L error", motor[0]->GetClosedLoopError());
    frc::SmartDashboard::PutNumber("motor_R error", motor[1]->GetClosedLoopError());
    frc::SmartDashboard::PutNumber("motor_L vel", motor[0]->GetSelectedSensorVelocity());
    frc::SmartDashboard::PutNumber("motor_R vel", motor[1]->GetSelectedSensorVelocity());
    frc::SmartDashboard::PutNumber("motor_L current", motor[0]->GetOutputCurrent());
    frc::SmartDashboard::PutNumber("motor_R current", motor[1]->GetOutputCurrent());
}
#endif