#include "dials.h"
Dials::Dials(int deviceNumber)
{
    i2cPort = frc::I2C::Port::kOnboard;
    m_colorSensor = new rev::ColorSensorV3(i2cPort);
    set_color_rgb();
    can_id = deviceNumber;
    motor = new TalonFX(can_id);
    //TODO: 设置位置模式
    fx_motor_magic(0.3,0.1,0);

}

Dials::~Dials()
{
  delete m_colorSensor;
  delete motor;
}

///< 设置颜色对应的rgb值
void Dials::set_color_rgb(void)
{
    color_target[B] = frc::Color(0.143, 0.427, 0.429);
    color_target[G] = frc::Color(0.197, 0.561, 0.240);
    color_target[R] = frc::Color(0.561, 0.232, 0.114);
    color_target[Y] = frc::Color(0.361, 0.524, 0.113);
    for(int i = 0;i<ALL_COLOR;i++)
        m_colorMatcher.AddColorMatch(color_target[i]);
}
Dials::COLOR Dials::get_color(void)
{
    detectedColor = m_colorSensor->GetColor();
    frc::Color matchedColor = m_colorMatcher.MatchClosestColor(detectedColor, confidence);
    for(int i = 0;i<ALL_COLOR;i++)
    {
        if(matchedColor == color_target[i])
        {
            curr_color = COLOR(i);
            return curr_color;
        }
    }
    curr_color = ALL_COLOR;
    return ALL_COLOR;
}
///< 旋转控制
//TODO: 待测试整个流程  
//TODO: 线程函数待写
void Dials::spin_control(float numb)
{
  if(!spin_control_thread)
  {
    spin_control_thread = true;
    color_sequence_pre = get_color();
    color_tran_count = 0;
    curr_position = motor->GetSelectedSensorPosition();
    motor->ConfigMotionCruiseVelocity(spin_control_vel, 10);
    c_numb_serson_return = c_numb_serson(numb + spin_numb_comp);
    motor->Set(ControlMode::MotionMagic,c_numb_serson_return + curr_position);    
    while (!is_finished_spin_control)
    {
      /* code */
      //TODO: 写颜色累计
      COLOR color = get_color();
      spin_pos_error = motor->GetClosedLoopError();
      if(color_sequence_check(color))
        color_tran_count++;
      if(color_tran_count >= ALL_COLOR * 2 * numb - 1)
        is_finished_spin_control = true;
      else if(spin_pos_error < is_finished_spin_pos_err)
      {
        curr_position = motor->GetSelectedSensorPosition();
        c_numb_serson_return = c_numb_serson((1/float(ALL_COLOR)) + spin_numb_comp);
        motor->Set(ControlMode::MotionMagic,c_numb_serson_return + curr_position);    
      }
      else
      {
        std::cout<<"spin_control"<<std::endl;
      }
    }
    
  }
  else
  {
    spin_control_thread = false;
  }
  


}
///< 颜色传感器联系校验
//TODO: 待验证 连续颜色突然插入不连续颜色
// 注意: 使用前要先清上次的颜色
bool Dials::color_sequence_check(COLOR curr)
{
  if(int(curr + 1)%int(ALL_COLOR) == int(color_sequence_pre) || int(curr - 1+ ALL_COLOR)%int(ALL_COLOR) == int(color_sequence_pre))
  {
    color_sequence_pre = curr;
    return true;
  }
  else return false;

}
// ///< 旋转转盘到指定颜色
// bool Dials::rotate_dials(COLOR color)
// {
//   curr_position = motor->GetSelectedSensorPosition();
  

// }
///< 装盘圈数转换猎鹰电机编码器位置  num 0~5 return 0 ~ 108850(5* 2048 * (810 / 76.2))
 int Dials::c_numb_serson(float numb)
 {
   return numb * serson * reduction;
 }
///< 获取转盘旋转是否完成
bool Dials::spin_control_is_is_finished(void)
{
  return is_finished_spin_control;
}

///< 装盘初始化
//TODO: 参数待调
///< 猎鹰电机位置模式初始化
void Dials::fx_motor_magic(float kf,float kp,float kd)
{
    motor->ConfigFactoryDefault();

    /* Configure Sensor Source for Pirmary PID */
    motor->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor,
                                        0, 
                                        10);

    motor->SetInverted(TalonFXInvertType::CounterClockwise);

    /* Set relevant frame periods to be at least as fast as periodic rate */
    motor->SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
    motor->SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);

    /* Set the peak and nominal outputs */
    motor->ConfigNominalOutputForward(0, 10);
    motor->ConfigNominalOutputReverse(0, 10);
    motor->ConfigPeakOutputForward(1, 10);
    motor->ConfigPeakOutputReverse(-1, 10);

    /* Set Motion Magic gains in slot0 - see documentation */
    motor->SelectProfileSlot(0, 0);
    motor->Config_kF(0, kf, 10);
    motor->Config_kP(0, kp, 10);
    motor->Config_kI(0, 0.0, 10);
    motor->Config_kD(0, kd, 10);

    /* Set acceleration and vcruise velocity - see documentation */
    motor->ConfigMotionCruiseVelocity(1500, 10);
    motor->ConfigMotionAcceleration(1500, 10);

    /* Zero the sensor */
    motor->SetSelectedSensorPosition(0, 0, 10);
    motor->ConfigMotionSCurveStrength(2, 0);//平滑

}


#ifdef DIALS_DEBUG
void Dials::display()
{
    frc::SmartDashboard::PutNumber("dials_d:", dials_d);
    frc::SmartDashboard::PutNumber("color_angle:", color_angle);
    frc::SmartDashboard::PutNumber("curr_position", curr_position);
    frc::SmartDashboard::PutNumber("dials_perimeter", dials_perimeter);
    frc::SmartDashboard::PutNumber("arc_length", arc_length);
    frc::SmartDashboard::PutNumber("spin_numb_comp", spin_numb_comp);
    frc::SmartDashboard::PutNumber("frictiongear_d", frictiongear_d);
    frc::SmartDashboard::PutNumber("frictiongear_d", c_numb_serson_return);
    frc::SmartDashboard::PutNumber("is_finished_spin_control", is_finished_spin_control);
    frc::SmartDashboard::PutNumber("spin_control_thread", spin_control_thread);
}
// ///< 调试时设置参数
void Dials::set_para()
{
  double comp = frc::SmartDashboard::GetNumber("spin_numb_comp", spin_numb_comp);
  if((comp != spin_numb_comp)) { spin_numb_comp = comp; }

}

#endif

