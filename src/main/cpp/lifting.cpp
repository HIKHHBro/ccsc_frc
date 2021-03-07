#include "lifting.h"
#include "status_led.h"
Lifting::Lifting(int id)
{
    set_reduction_ratiop(1,100);//1为位移输入,100为电机输出
    set_dia(45.3);//直径50mm
    set_loop_time(10000);
    reset_sw[0] = new frc::DigitalInput(4);//0通道
    reset_sw[1] = new frc::DigitalInput(5);//0通道
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
        motor[i]->Config_kF(0, reset_kf, 10);
        motor[i]->Config_kP(0, reset_kp, 10);
        motor[i]->Config_kI(0, 0.0, 10);
        motor[i]->Config_kD(0, 0.0, 10);

        /* Set acceleration and vcruise velocity - see documentation */
        motor[i]->ConfigMotionCruiseVelocity(5600, 10);
        motor[i]->ConfigMotionAcceleration(9000, 10);

        /* Zero the sensor */
        motor[i]->SetSelectedSensorPosition(0, 0, 10);
        motor[i]->ConfigMotionSCurveStrength(smoothing, 0);
        motor[i]->SetNeutralMode(NeutralMode::Brake);
        motor[i]->ConfigNeutralDeadband(0,10);
    }
}           


Lifting::~Lifting()
{
    delete reset_sw[0];
    delete reset_sw[1];

}
///< 设置电机执行
void Lifting::set_point(float len)
{
    if(is_reseted == true)
    {
        motor[0]->Set(ControlMode::MotionMagic, (mm_to_enc(len) + len_comp) * -1);
        motor[1]->Set(ControlMode::MotionMagic, mm_to_enc(len) + len_comp);
    }
    else
    {
        frc::SmartDashboard::PutString("status","Not reset");
        std::cout<<"Not reset"<<std::endl;
        // Status_led::set_tip_mode(Status_led::NO_Reset);
    }

}
///< 提升
bool Lifting::lift()
{
    return carry_out(lift_high,lift_speed);
}
///< 伸出
bool Lifting::stretch_out()
{
    return carry_out(route,stretch_speed);
}
///< 动作执行 s 位移 v 速度
float vvv1[2],accc1,accc2;
bool Lifting::carry_out(float s,float v)
{
    float error[2],v_temp[2];
    motor[0]->ConfigMotionCruiseVelocity(rpm_to_enc_100ms(v), 10);
    motor[1]->ConfigMotionCruiseVelocity(rpm_to_enc_100ms(v), 10);
    for(int i = 0;i<M_ALL;i++)
    {
        error[i] = get_position_error(mm_to_enc(s),motor[i]->GetSelectedSensorPosition());
        vvv1[i] = v_temp[i] = motor[i]->GetSelectedSensorVelocity();
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
///< 复位
bool Lifting::reset()
{
    start_detach();
    return is_reseted;
}
///< 获取复位按键值 松开是高电平，按下是低电平
bool Lifting::get_reset_key(MOTOR M)
{
    return !reset_sw[M]->Get();
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
//TODO: 添加失败是报详细左右的错误log
//TODO: 增加log信息
//TODO: 复位的单位仍然是原始单位 待转换
int aa = 1;
void Lifting::run()
{
    for(int i = 0;i<M_ALL;i++)
    {
        reset_flag[i] = -1;
        reset_flag[i] = -1;
        reset_error_count[i] = 0;
    }
    is_reseted = false;
    while (!isInterrupted())
    {
        for(int i = 0;i<M_ALL;i++)
        {
            if(i == 0)
            {
                aa = -1;
            }
            else
            {
                aa = 1;
            }
            if(get_reset_key(MOTOR(i)))
            {
                motor[i]->SetSelectedSensorPosition(0, 0, 10);
                motor[i]->Set(ControlMode::PercentOutput,0);
                reset_flag[i] = 1;
            }
            else
            {
                motor[i]->Set(ControlMode::PercentOutput,reset_speed * aa);
            }
        }
        if(reset_flag[0] == 1 && reset_flag[1] == 1)
        {
            is_reseted = true;
            interrupt();
        }
        thread_sleep();
    }
    interrupt();
    disable_motor();
}
///< 失能电机
//TODO: 当前位置赋值模式不能用这个去停止，因为停止需要一个减速过程和系统时间延迟，所以导致位置误差始终保持一定值导致无法停止 
//TODO: 速度模式会急刹
//TODO: 待优化
void Lifting::disable_motor()
{
    motor[0]->Set(ControlMode::PercentOutput,0);
    motor[1]->Set(ControlMode::PercentOutput,0);
}
///< 获取复位状态
bool Lifting::get_reset_status()
{
    return is_reseted;
}
#ifdef LIFT_DEBUG
void Lifting::display()
{
    // frc::SmartDashboard::PutNumber("smoothing",smoothing);
    // frc::SmartDashboard::PutNumber("len_comp",len_comp);
    frc::SmartDashboard::PutNumber("route",route);
    frc::SmartDashboard::PutNumber("lift_high",lift_high);
    // frc::SmartDashboard::PutNumber("pos_thres",pos_thres);
    // frc::SmartDashboard::PutNumber("speed_thres",speed_thres);
    frc::SmartDashboard::PutNumber("get_stretch_speed",stretch_speed);
    frc::SmartDashboard::PutNumber("get_lift_speed",lift_speed);
    frc::SmartDashboard::PutNumber("reset_speed",reset_speed);
    // frc::SmartDashboard::PutNumber("reset_output",reset_output);
    // frc::SmartDashboard::PutNumber("reset_current_thres",reset_current_thres);
    // frc::SmartDashboard::PutNumber("get_acc",acc);
    // frc::SmartDashboard::PutNumber("get_reset_acc",reset_acc);
    // frc::SmartDashboard::PutNumber("复位速度阈值",reset_speed_thres);
    frc::SmartDashboard::PutNumber("set_kp",kp);
    frc::SmartDashboard::PutNumber("set_kf",kf);
    // frc::SmartDashboard::PutNumber("set_reset_kp",reset_kp);
    // frc::SmartDashboard::PutNumber("set_reset_kf",reset_kf);
    // frc::SmartDashboard::PutNumber("set_reset_error_thre",reset_error_thre);
}
void Lifting::debug()
{

    // float get1 = frc::SmartDashboard::GetNumber("smoothing",smoothing);
    // if(get1 != smoothing) {smoothing = get1;}
    // float get2 = frc::SmartDashboard::GetNumber("len_comp",len_comp);
    // if(get2 != len_comp) {len_comp = get2;}
    float get3 = frc::SmartDashboard::GetNumber("route",route);
    if(get3 != route) {route = get3;}
    float get4 = frc::SmartDashboard::GetNumber("lift_high",lift_high);
    if(get4 != lift_high) {lift_high = get4;}
    // float get5 = frc::SmartDashboard::GetNumber("pos_thres",pos_thres);
    // if(get5 != pos_thres) {pos_thres = get5;}
    // float get6 = frc::SmartDashboard::GetNumber("speed_thres",speed_thres);
    // if(get6 != speed_thres) {speed_thres = get6;}
    // float get7 = frc::SmartDashboard::GetNumber("get_stretch_speed",stretch_speed);
    // if(get7 != stretch_speed) {stretch_speed = get7;}
    // int get8 = frc::SmartDashboard::GetNumber("is_stretched",is_stretched);
    // if(get8 != is_stretched) {is_stretched = get8;}
    // float get9 = frc::SmartDashboard::GetNumber("reset_speed",reset_speed);
    // if(get9 != reset_speed) {reset_speed = get9;}
    // float get10 = frc::SmartDashboard::GetNumber("reset_output",reset_output);
    // if(get10 != reset_output) {reset_output = get10;}
    // float get11 = frc::SmartDashboard::GetNumber("reset_current_thres",reset_current_thres);
    // if(get11 != reset_current_thres) {reset_current_thres = get11;}

    // float get12 = frc::SmartDashboard::GetNumber("get_lift_speed",lift_speed);
    // if(get12 != lift_speed) {lift_speed = get12;}

    // float get13 = frc::SmartDashboard::GetNumber("get_acc",acc);
    // if(get13 != acc)
    // {
    //     acc = limit(get13,0.0,3000.0);
    //     motor[0]->ConfigMotionAcceleration(acc, 10);
    //     motor[1]->ConfigMotionAcceleration(acc, 10);
    // }

    // float get14 = frc::SmartDashboard::GetNumber("复位速度阈值",reset_speed_thres);
    // if(get14 != reset_speed_thres)
    // {
    //     reset_speed_thres = limit(reset_speed_thres,0.0,1000.0);
    // }

    // float get15 = frc::SmartDashboard::GetNumber("get_reset_acc",reset_acc);
    // if(get15 != reset_acc)
    // {
    //     reset_acc = limit(get15,0.0,3000.0);
    //     motor[0]->ConfigMotionAcceleration(reset_acc, 10);
    //     motor[1]->ConfigMotionAcceleration(reset_acc, 10);
    // }

    double Get16  = frc::SmartDashboard::GetNumber("set_kp",kp);
    if((Get16 != kp))
    {
        kp = Get16;
        motor[0]->Config_kP(0,kp,10);
        motor[1]->Config_kP(0,kp,10);
    }

    double Get17  = frc::SmartDashboard::GetNumber("set_kf",kf);
    if((Get17 != kf))
    {
        kf = Get17;
        motor[0]->Config_kF(0,kf,10);
        motor[1]->Config_kF(0,kf,10);
    }

    // double Get18  = frc::SmartDashboard::GetNumber("set_reset_kp",reset_kp);
    // if((Get18 != kp))
    // {
    //     reset_kp = Get18;
    //     motor[0]->Config_kP(0,reset_kp,10);
    //     motor[1]->Config_kP(0,reset_kp,10);
    // }

    // double Get19  = frc::SmartDashboard::GetNumber("set_reset_kf",reset_kf);
    // if((Get19 != reset_kf))
    // {
    //     reset_kf = Get19;
    //     motor[0]->Config_kF(0,reset_kf,10);
    //     motor[1]->Config_kF(0,reset_kf,10);
    // }

    // float get20 = frc::SmartDashboard::GetNumber("set_reset_error_thre",reset_error_thre);
    // if(get20 != reset_error_thre) {reset_error_thre = get20;}

    frc::SmartDashboard::PutNumber("motor_L error", get_position_error(mm_to_enc(lift_high),motor[0]->GetSelectedSensorPosition()));
    frc::SmartDashboard::PutNumber("motor_R error", get_position_error(mm_to_enc(lift_high),motor[1]->GetSelectedSensorPosition()));
    frc::SmartDashboard::PutNumber("motor_L vel", enc_100ms_to_rpm(motor[0]->GetSelectedSensorVelocity()));
    frc::SmartDashboard::PutNumber("motor_R vel", enc_100ms_to_rpm(motor[1]->GetSelectedSensorVelocity()));
    frc::SmartDashboard::PutNumber("motor_L current", motor[0]->GetOutputCurrent());
    frc::SmartDashboard::PutNumber("motor_R current", motor[1]->GetOutputCurrent());
    // frc::SmartDashboard::PutNumber("reset loop hz", get_loop_freq());
    frc::SmartDashboard::PutNumber("lift_status", get_status());
    // frc::SmartDashboard::PutNumber("motor L ID", motor[0]->GetDeviceID());
    // frc::SmartDashboard::PutNumber("motor R ID", motor[1]->GetDeviceID());
    frc::SmartDashboard::PutNumber("motor L pos", motor[0]->GetSelectedSensorPosition());
    frc::SmartDashboard::PutNumber("motor R pos", motor[1]->GetSelectedSensorPosition());
    frc::SmartDashboard::PutNumber("motor target pos", mm_to_enc(lift_high) + len_comp);
    // frc::SmartDashboard::PutNumber("rpm_to_enc_100ms(v)", rpm_to_enc_100ms(stretch_speed));
    frc::SmartDashboard::PutNumber("motor_L vel 100ms", motor[0]->GetSelectedSensorVelocity());
    frc::SmartDashboard::PutNumber("motor_R vel 100ms", motor[1]->GetSelectedSensorVelocity());


    frc::SmartDashboard::PutNumber("get_reset_keyL", get_reset_key(M1));
    frc::SmartDashboard::PutNumber("get_reset_keyR", get_reset_key(M2));
    // frc::SmartDashboard::PutNumber("reset_acc enc", reset_acc);
    // frc::SmartDashboard::PutNumber("acc_enc", acc);
    frc::SmartDashboard::PutNumber("复位标志", is_reseted);
    frc::SmartDashboard::PutNumber("左复位标志", reset_flag[M1]);
    frc::SmartDashboard::PutNumber("右复位标志", reset_flag[M2]);
    // frc::SmartDashboard::PutNumber("左复位堵转累计记数", reset_error_count[M1]);
    // frc::SmartDashboard::PutNumber("右复位堵转累计记数", reset_error_count[M2]);

    thread_debug();

    
}
#endif

//TODO: 速度和加速度单位，刻度/100ms -> RPM -> mm/s
