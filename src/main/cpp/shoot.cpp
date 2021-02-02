#include "shoot.h"
Shoot::Shoot(int pwm_c,int can_id)//0,12
{
  for(int i = 0;i<ALL;i++)
  {
    motor[i] = new Neo(pwm_c+i,acc[i]);
  }

    gimbal_motor = new TalonFX(can_id);  
    set_reduction_ratiop(126,1);

    /* Factory default hardware to prevent unexpected behavior */
    gimbal_motor->ConfigFactoryDefault();

    /* Configure Sensor Source for Pirmary PID */
    gimbal_motor->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor,0, 10);

    gimbal_motor->SetInverted(TalonFXInvertType::CounterClockwise);

    /* Set relevant frame periods to be at least as fast as periodic rate */
    gimbal_motor->SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
    gimbal_motor->SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);

    /* Set the peak and nominal outputs */
    gimbal_motor->ConfigNominalOutputForward(0, 10);
    gimbal_motor->ConfigNominalOutputReverse(0, 10);
    gimbal_motor->ConfigPeakOutputForward(1, 10);
    gimbal_motor->ConfigPeakOutputReverse(-1, 10);

    /* Set Motion Magic gains in slot0 - see documentation */
    gimbal_motor->SelectProfileSlot(0, 0);
    gimbal_motor->Config_kF(0, kf, 10);
    gimbal_motor->Config_kP(0, kp, 10);
    gimbal_motor->Config_kI(0, 0.0, 10);
    gimbal_motor->Config_kD(0, 0.0, 10);

    /* Set acceleration and vcruise velocity - see documentation */
    gimbal_motor->ConfigMotionCruiseVelocity(1500, 10);
    gimbal_motor->ConfigMotionAcceleration(1500, 10);

    /* Zero the sensor */
    gimbal_motor->SetSelectedSensorPosition(0, 0, 10);
    gimbal_motor->ConfigMotionSCurveStrength(smoothing, 0);
    gimbal_motor->SetNeutralMode(Brake);
    gimbal_motor->ConfigNeutralDeadband(0,10);

}
Shoot::~Shoot()
{
  for(int i = 0;i<ALL;i++)
    delete motor[i];
  delete gimbal_motor;
}
///< 开启竖直传送
//TODO: 待测试
bool Shoot::open_vertical_transfer()
{
  carry_out(Ver_tr,neo_speed[Ver_tr]);
  if(motor[Ver_tr]->is_complete_acc(neo_speed[Ver_tr]))
    return true;
  else return false;
}

///< 停止竖直传送
//TODO: 待测试
bool Shoot::stop_vertical_transfer()
{
  carry_out(Ver_tr,-neo_speed[Ver_tr]);
  if(motor[Ver_tr]->is_complete_acc(-neo_speed[Ver_tr]))
    return true;
  else return false;
}

///< 开启水平传送
void Shoot::open_horizontal_transfer()
{
  //  carry_out(Hor_tr,0.30);
   carry_out(Hor_tr_u,0.10);
}
///< 关闭竖直传送
void Shoot::close_vertical_transfer()
{
  carry_out(Ver_tr,0);
}
///< 关闭水平传送
void Shoot::close_horizontal_transfer()
{
  carry_out(Hor_tr,0);
  carry_out(Hor_tr_u,0);

}
///< neo电机动作执行
//TODO: 待测试rpm_to_per转换是否有误
void Shoot::carry_out(MOTOR M,float rpm)
{
  motor[M]->Set(rpm);
}
///< 设置云台转动角度 下限位为0度,向上转angle度,
//TODO: 待测四
void Shoot::set_gimbal_angle(float angle)
{
  gimbal_motor->Set(ControlMode::MotionMagic,angle_to_enc(limit(angle,0.0,max_angle)));
}
///< 
///< 云台初始化校准线程函数
//TODO: 待测试
void Shoot::run()
{

    gimbal_motor->ConfigClosedLoopPeakOutput(0,reset_output,0);    
    reset_error_count = 0;
    is_reseted = false;
    while (!isInterrupted())
    {
        if(gimbal_motor->GetOutputCurrent() > reset_current_thres &&\
           IS_X_SECTION(gimbal_motor->GetSelectedSensorVelocity(),reset_speed_thres))
        {
            if(reset_error_count < reset_error_thre)
            {
                gimbal_motor->Set(ControlMode::Velocity,reset_speed/10);
                reset_error_count++;
            }
            else
            {
                gimbal_motor->SetSelectedSensorPosition(0, 0, 10);
                is_reseted = true;
                interrupt();
            }
        }
        else
        {
            gimbal_motor->Set(ControlMode::Velocity,reset_speed);
        }
        usleep(reset_period);
    }
    interrupt();
}





bool Shoot::reset()
{
    start_join();
    return is_reseted;
}
//TODO: 摄像头待写


///< 开启发射
void Shoot::start_shoot()
{
  
  open_vertical_transfer();
  carry_out(Sh1,neo_speed[Sh1]);
  carry_out(Sh2,neo_speed[Sh2]);
}
///< 停止发射
///< 开启发射
void Shoot::stop_shoot()
{
  stop_vertical_transfer();
  carry_out(Sh1,0);
  carry_out(Sh2,0);
}
#ifdef SHOOT_DEBUG
void Shoot::display()
{
frc::SmartDashboard::PutNumber("set_neo_speed Hor_tr",neo_speed[Hor_tr]);
frc::SmartDashboard::PutNumber("set_neo_speed Ver_tr",neo_speed[Ver_tr]);
frc::SmartDashboard::PutNumber("set_neo_speed Sh1",neo_speed[Sh1]);
frc::SmartDashboard::PutNumber("set_neo_speed Sh2",neo_speed[Sh2]);
frc::SmartDashboard::PutNumber("set_gimbal_kp",kp);
frc::SmartDashboard::PutNumber("set_gimbal_kf",kf);
frc::SmartDashboard::PutNumber("set_gimbal_smoothing",smoothing);
frc::SmartDashboard::PutNumber("gimbal max angle",max_angle);
frc::SmartDashboard::PutNumber("get_reset_output",reset_output);
frc::SmartDashboard::PutNumber("get_reset_speed_thres",reset_speed_thres);

}
void Shoot::debug() 
{
    double Get1  = frc::SmartDashboard::GetNumber("set_neo_speed Hor_tr",neo_speed[Hor_tr]);
    if((Get1 != neo_speed[Hor_tr])) { neo_speed[Hor_tr] = Get1;}

    double Get2  = frc::SmartDashboard::GetNumber("set_neo_speed Ver_tr",neo_speed[Ver_tr]);
    if((Get2 != neo_speed[Ver_tr])) { neo_speed[Ver_tr] = Get2;}

    double Get3  = frc::SmartDashboard::GetNumber("set_neo_speed Sh1",neo_speed[Sh1]);
    if((Get3 != neo_speed[Sh1])) { neo_speed[Sh1] = Get3;}

    double Get4  = frc::SmartDashboard::GetNumber("set_neo_speed Sh2",neo_speed[Sh2]);
    if((Get4 != neo_speed[Sh2])) { neo_speed[Sh2] = Get4;}

    double Get5  = frc::SmartDashboard::GetNumber("set_gimbal_kp",kp);
    if((Get5 != kp)) { kp = Get5; gimbal_motor->Config_kP(0,kp,10);}

    double Get6  = frc::SmartDashboard::GetNumber("set_gimbal_kf",kf);
    if((Get6 != kf)) { kf = Get6; gimbal_motor->Config_kP(0,kf,10);}

    double Get7  = frc::SmartDashboard::GetNumber("set_gimbal_smoothing",smoothing);
    if((Get7 != smoothing)) { smoothing = Get7; gimbal_motor->ConfigMotionSCurveStrength(smoothing, 0);}

    double Get8  = frc::SmartDashboard::GetNumber("get_reset_output",reset_output);
    if((Get8 != reset_output)) { reset_output = Get8;}

    double Get9  = frc::SmartDashboard::GetNumber("get_reset_speed_thres",reset_speed_thres);
    if((Get9 != reset_speed_thres)) { reset_speed_thres = Get9;}

    frc::SmartDashboard::PutNumber("gimbal angle",enc_to_angle(gimbal_motor->GetSelectedSensorPosition()));
    frc::SmartDashboard::PutNumber("gimbal encoder",gimbal_motor->GetSelectedSensorPosition());


}
#endif