#include "lifting.h"
Lifting::Lifting(int id)
{
    set_reduction_ratiop(1,100);//1为位移输入,100为电机输出
    set_dia(50);//直径50mm
    set_loop_time(50000);//50ms
    reset_sw[0] = new frc::DigitalInput(0);//0通道
    reset_sw[1] = new frc::DigitalInput(1);//0通道
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
        motor[i]->Config_kF(0, 0.2, 10);
        motor[i]->Config_kP(0, 0.1, 10);
        motor[i]->Config_kI(0, 0.0, 10);
        motor[i]->Config_kD(0, 0.0, 10);

        /* Set acceleration and vcruise velocity - see documentation */
        motor[i]->ConfigMotionCruiseVelocity(1500, 10);
        motor[i]->ConfigMotionAcceleration(1500, 10);

        /* Zero the sensor */
        motor[i]->SetSelectedSensorPosition(0, 0, 10);
        motor[i]->ConfigMotionSCurveStrength(smoothing, 0);
        motor[i]->SetNeutralMode(Brake);
        motor[i]->ConfigNeutralDeadband(0,10);
    }
    //TODO: 待测试跟随
    // motor[0]->Follow(*motor[1],FollowerType::FollowerType_AuxOutput1);
}           


Lifting::~Lifting()
{
    delete reset_sw[0];
    delete reset_sw[1];

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
    return carry_out(lift_high,stretch_speed);
}
//TODO: 待测试
///< 伸出
bool Lifting::stretch_out()
{
    return carry_out(route,stretch_speed);
}
///< 动作执行 s 位移 v 速度
bool Lifting::carry_out(float s,float v)
{
    float error[2],v_temp[2];
    motor[0]->ConfigMotionCruiseVelocity(rpm_to_enc_100ms(v), 10);
    motor[1]->ConfigMotionCruiseVelocity(rpm_to_enc_100ms(v), 10);
    for(int i = 0;i<M_ALL;i++)
    {
        error[i] = get_position_error(mm_to_enc(s),motor[i]->GetSelectedSensorPosition());
        v_temp[i] = motor[i]->GetSelectedSensorVelocity();
    }
    if(IS_X_SECTION(error[0],pos_thres) && \
        IS_X_SECTION(error[1],pos_thres) &&\
        v_temp[0] == 0 && \
        v_temp[1] == 0)
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
    return carry_out(0,stretch_speed);
}
//TODO：线程函数
///< 复位
bool Lifting::reset()
{
    start_join();
    return is_reseted;
}
//TODO: 待测试
///< 获取复位按键值
bool Lifting::get_reset_key(MOTOR M)
{
    return reset_sw[M]->Get();
}

///< 获取伸缩杆状态
Lifting::STATUS Lifting::get_status()
{
    if(IS_X_SECTION(get_position_error(mm_to_enc(route),motor[0]->GetSelectedSensorPosition()),pos_thres) &&\
       IS_X_SECTION(get_position_error(mm_to_enc(route),motor[1]->GetSelectedSensorPosition()),pos_thres))
    {
        lift_status = Stretched;
    }
    else if(IS_X_SECTION(get_position_error(0,motor[0]->GetSelectedSensorPosition()),pos_thres) &&\
            IS_X_SECTION(get_position_error(0,motor[1]->GetSelectedSensorPosition()),pos_thres))
    {
        lift_status = Shrinked;
    }
    else
    {
        lift_status = Mid;
    }
    return lift_status;
}
///< 复位线程
//TODO: 待测试
void Lifting::run()
{

    motor[0]->ConfigClosedLoopPeakOutput(0,reset_output,0);
    motor[1]->ConfigClosedLoopPeakOutput(0,reset_output,0);
    while (!isInterrupted())
    {
        if(get_reset_key(M1))
        {
            if(motor[0]->GetOutputCurrent() > reset_current_thres &&\
               motor[0]->GetSelectedSensorVelocity()==0 &&\
               motor[1]->GetOutputCurrent() > reset_current_thres &&\
               motor[1]->GetSelectedSensorVelocity()==0
             )
             {
                 motor[0]->SetSelectedSensorPosition(0, 0, 10);
                 motor[1]->SetSelectedSensorPosition(0, 0, 10);
                 is_reseted = true;
                 reset_error_count =0;
                 interrupt();
             }
             else
             {
                if(reset_error_count < reset_error_thre)
                {
                    motor[0]->Set(ControlMode::Velocity,reset_speed);
                    motor[1]->Set(ControlMode::Velocity,reset_speed);
                    reset_error_count++;
                }
                else
                {
                    motor[0]->SetSelectedSensorPosition(0, 0, 10);
                    motor[1]->SetSelectedSensorPosition(0, 0, 10);
                    error.append("reset fail - Check whether the lift reset light touch switch is damaged -\n");
                    motor[0]->Set(ControlMode::Velocity,0);
                    motor[1]->Set(ControlMode::Velocity,0);
                    is_reseted = false;
                    interrupt();
                }
             }

        }
        else if(motor[0]->GetOutputCurrent() > reset_current_thres &&\
               motor[0]->GetSelectedSensorVelocity()==0 &&\
               motor[1]->GetOutputCurrent() > reset_current_thres &&\
               motor[1]->GetSelectedSensorVelocity()==0
             )
        {
            motor[0]->SetSelectedSensorPosition(0, 0, 10);
            motor[1]->SetSelectedSensorPosition(0, 0, 10);
            error.append("reset fail - Check whether the lift reset light touch switch is damaged -\n");
            motor[0]->Set(ControlMode::Velocity,0);
            motor[1]->Set(ControlMode::Velocity,0);
            is_reseted = false;
            interrupt();
        }
        else
        {
            motor[0]->Set(ControlMode::Velocity,reset_speed);
            motor[1]->Set(ControlMode::Velocity,reset_speed);
        }
    }
}
///< 失能电机
//TODO: 不能用这个去停止，因为停止需要一个减速过程和系统时间延迟，所以导致位置误差始终保持一定值导致无法停止 
void Lifting::disable_motor()
{
    motor[0]->Set(ControlMode::Velocity,0);
    motor[1]->Set(ControlMode::Velocity,0);
}
///< 调试用遥控控制获取电机所需运行的位移
// bool Lifting::debug_get_para()
// {
    
// }
///< 获取复位状态
//TODO: 待写
bool Lifting::get_reset_status()
{
    return is_reseted;
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
 frc::SmartDashboard::PutNumber("get_stretch_speed",stretch_speed);
 frc::SmartDashboard::PutNumber("get_shrink_speed",shrink_speed);
 frc::SmartDashboard::PutNumber("reset_speed",reset_speed);
 frc::SmartDashboard::PutNumber("reset_output",reset_output);
 frc::SmartDashboard::PutNumber("reset_current_thres",reset_current_thres);
 frc::SmartDashboard::PutNumber("get_acc",acc);
 frc::SmartDashboard::PutNumber("get_reset_acc",reset_acc);
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
    int get7 = frc::SmartDashboard::GetNumber("get_stretch_speed",stretch_speed);
    if(get7 != stretch_speed) {stretch_speed = get7;}
    // int get8 = frc::SmartDashboard::GetNumber("is_stretched",is_stretched);
    // if(get8 != is_stretched) {is_stretched = get8;}
    int get9 = frc::SmartDashboard::GetNumber("reset_speed",reset_speed);
    if(get9 != reset_speed) {reset_speed = get9;}
    int get10 = frc::SmartDashboard::GetNumber("reset_output",reset_output);
    if(get10 != reset_output) {reset_output = get10;}
    int get11 = frc::SmartDashboard::GetNumber("reset_current_thres",reset_current_thres);
    if(get11 != reset_current_thres) {reset_current_thres = get11;}

    int get12 = frc::SmartDashboard::GetNumber("get_shrink_speed",shrink_speed);
    if(get12 != shrink_speed) {shrink_speed = get12;}

    int get13 = frc::SmartDashboard::GetNumber("get_acc",acc);
    if(get13 != acc)
    {
        acc = get13;
        acc = limit(acc,0.0,3000.0);
        motor[0]->ConfigMotionAcceleration(mm_to_enc(acc), 10);
        motor[1]->ConfigMotionAcceleration(mm_to_enc(acc), 10);
    }//TODO: 待测试

    int get15 = frc::SmartDashboard::GetNumber("get_reset_acc",reset_acc);
    if(get15 != reset_acc) {reset_acc = get15;}

    frc::SmartDashboard::PutNumber("motor_L error", get_position_error(mm_to_enc(lift_high),motor[0]->GetSelectedSensorPosition()));
    frc::SmartDashboard::PutNumber("motor_R error", get_position_error(mm_to_enc(lift_high),motor[1]->GetSelectedSensorPosition()));
    frc::SmartDashboard::PutNumber("motor_L vel", enc_100ms_to_rpm(motor[0]->GetSelectedSensorVelocity()));
    frc::SmartDashboard::PutNumber("motor_R vel", enc_100ms_to_rpm(motor[1]->GetSelectedSensorVelocity()));
    frc::SmartDashboard::PutNumber("motor_L current", motor[0]->GetOutputCurrent());
    frc::SmartDashboard::PutNumber("motor_R current", motor[1]->GetOutputCurrent());
    frc::SmartDashboard::PutNumber("reset loop hz", get_loop_freq());
    frc::SmartDashboard::PutNumber("lift_status", get_status());
    frc::SmartDashboard::PutNumber("motor L ID", motor[0]->GetDeviceID());
    frc::SmartDashboard::PutNumber("motor R ID", motor[1]->GetDeviceID());
    frc::SmartDashboard::PutNumber("motor L pos", motor[0]->GetSelectedSensorPosition());
    frc::SmartDashboard::PutNumber("motor R pos", motor[1]->GetSelectedSensorPosition());
    frc::SmartDashboard::PutNumber("motor target pos", mm_to_enc(lift_high) + len_comp);
    frc::SmartDashboard::PutNumber("rpm_to_enc_100ms(v)", rpm_to_enc_100ms(stretch_speed));
    frc::SmartDashboard::PutNumber("motor_L vel 100ms", motor[0]->GetSelectedSensorVelocity());
    frc::SmartDashboard::PutNumber("motor_R vel 100ms", motor[1]->GetSelectedSensorVelocity());
}
#endif