#include "dials.h"
Dials::Dials(int deviceNumber)
{
    i2cPort = frc::I2C::Port::kOnboard;
    m_colorSensor = new rev::ColorSensorV3(i2cPort);
    set_color_rgb();
    set_reduction_ratiop(frictiongear_d,dials_d);//1为位移输入,100为电机输出
    set_dia(frictiongear_d);
    set_loop_time(5000);//50ms
    time_thre[0] = (int)(get_loop_freq())*wait_time[Spin];
    time_thre[1] = (int)(get_loop_freq())*wait_time[Pos];

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
///<开启旋转控制
bool Dials::start_spin_control(float numb)
{
    spin_numb = numb;
    state = Spin;
    start_detach();
    return spin_control_is_finished();
}

///<开启位置控制
void Dials::start_pos_control(COLOR color)
{
    target_color = color;
    state = Pos;
    start_detach();
}

///< 旋转控制
//TODO: 待测试整个流程  
//TODO: 线程函数待写
void Dials::spin_control_thread()
{
    color_sequence_pre = get_color();
    color_tran_count = 0;
    motor->SetSelectedSensorPosition(0, 0, 10);
    motor->ConfigMotionCruiseVelocity(spin_control_vel, 10);
    c_numb_serson_return = angle_to_enc((spin_numb + spin_numb_comp)*360);
    motor->Set(ControlMode::MotionMagic,c_numb_serson_return);   
    time_count[Spin] = 0; 
    is_finished_spin_control = false;
    while (!isInterrupted())
    {
      /* code */
      //TODO: 写颜色累计
      COLOR color = get_color();
      spin_pos_error = get_position_error(c_numb_serson_return,motor->GetSelectedSensorPosition());
      if(color_sequence_check(color))
        color_tran_count++;
      if(color_tran_count >= ALL_COLOR * 2 * spin_numb - 1)
      {
            if(time_count[Spin] < time_thre[Spin])
                time_count[Spin]++;
            else
            {
                is_finished_spin_control = true;
                interrupt();
            }

      }
      else if(abs(spin_pos_error) < is_finished_spin_pos_err)
      {
        curr_position = motor->GetSelectedSensorPosition();
        c_numb_serson_return = angle_to_enc(((1/float(ALL_COLOR)) + spin_numb_comp)*360);
        motor->Set(ControlMode::MotionMagic,c_numb_serson_return + curr_position);    
      }
      else
      {
        std::cout<<"spin_control"<<std::endl;
      }
      usleep(reset_period);
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
///< 选择旋转最优路径 返回角度值//顺时针 对应电机逆时针
float Dials::optimal_path(COLOR target,COLOR curr)
{
    int error =  curr - target;
    if(abs(error) > 3)
    {
        std::cout<<"Color recognition failure"<<std::endl;
        return 0;
    }
    if(abs(error) ==3)
    {
        return direction *(abs(error)/error)*color_angle;
    } 
    else
    {
        return error * color_angle;
    }
    
}
///< 旋转转盘到指定颜色
//TODO: 待测试
void Dials::pos_control_thread()
{
    color_sequence_pre = get_color();
    if(color_sequence_pre == ALL_COLOR)
    {
        interrupt();
        is_finished_pos_control = false;
    }    
    color_tran_count = 0;
    is_finished_pos_control = false;
    motor->SetSelectedSensorPosition(0, 0, 10);
    motor->ConfigMotionCruiseVelocity(spin_control_vel, 10);
    target_angle = optimal_path(target_color,color_sequence_pre);
    motor->Set(ControlMode::MotionMagic,angle_to_enc(target_angle));   
    time_count[Pos] = 0; 
    int color_numb = target_angle/color_angle;
    while (!isInterrupted())
    {
      /* code */
      //TODO: 写颜色累计
      COLOR color = get_color();
      spin_pos_error = get_position_error(c_numb_serson_return,motor->GetSelectedSensorPosition());
      if(color_sequence_check(color))
      {
          if(color_numb > 0)
          {
              color_tran_count++;
          }
          else 
          {
              color_tran_count--;
          }
      }
      if(abs(spin_pos_error) < is_finished_spin_pos_err)
      {
        if(abs(color_tran_count) < abs(color_numb))
        {
            curr_position = motor->GetSelectedSensorPosition();
            c_numb_serson_return = angle_to_enc((color_numb - color_tran_count)*360);
            motor->Set(ControlMode::MotionMagic,c_numb_serson_return + curr_position);  
        }
        else
        {
            if(time_count[Pos] < time_thre[Pos])
            {
                time_count[Pos]++;
            } 
            else
            {
                is_finished_pos_control = true;
                interrupt();
            }
        }
      }
      else
      {
          motor->Set(ControlMode::MotionMagic,angle_to_enc(target_angle)); 
      }
      usleep(reset_period);
    }
}

///< 获取转盘旋转是否完成
bool Dials::spin_control_is_finished(void)
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
///< 转盘线程函数
void Dials::run()
{
  switch (state)
  {
  case Spin:
    spin_control_thread();
    break;
  case Pos:
    pos_control_thread();
    break;
  
  default:
    break;
  }
  
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
    frc::SmartDashboard::PutNumber("spin_control_thread_status", spin_control_thread_status);
}
// ///< 调试时设置参数
void Dials::set_para()
{
  double comp = frc::SmartDashboard::GetNumber("spin_numb_comp", spin_numb_comp);
  if((comp != spin_numb_comp)) { spin_numb_comp = comp; }

}

#endif

