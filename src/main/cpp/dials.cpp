#include "dials.h"
#include <string>
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
    //设置位置模式
    fx_motor_magic(0.4,0.3,0);
    solenoid[0] = new frc::Solenoid(20,2);
    solenoid[1] = new frc::Solenoid(20,3);
    is_arrived_sw = new frc::DigitalInput(0);

}

Dials::~Dials()
{
  delete m_colorSensor;
  delete motor;
}

///< 设置颜色对应的rgb值
void Dials::set_color_rgb(void)
{
    color_target[B] = frc::Color(0.185669, 0.488647, 0.32605);
    color_target[G] = frc::Color(0.198364, 0.580444, 0.221313);
    color_target[R] = frc::Color(0.442261, 0.38269, 0.175171);
    color_target[Y] = frc::Color(0.287231, 0.557495, 0.154907);
    for(int i = 0;i<ALL_COLOR;i++)
        m_colorMatcher.AddColorMatch(color_target[i]);
}
Dials::COLOR Dials::get_color(void)
{
    detectedColor = m_colorSensor->GetColor();
    frc::Color matchedColor = m_colorMatcher.MatchClosestColor(detectedColor, confidence);
    for(int i = 0;i<ALL_COLOR;i++)
    {
        if((matchedColor == color_target[i]) && (confidence > 0.98))
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
  if(is_arrived())
  {
    spin_numb = numb;
    state = Spin;
    start_detach();
    return spin_control_is_finished();
  }
  else return false;
}

///<开启位置控制
void Dials::start_pos_control(COLOR color)
{
 if(is_arrived())
 {
    target_color = color;
    state = Pos;
    start_detach();
 }

}

///< 旋转控制
float ddd = 0;
float creddd = 0;
void Dials::spin_control_thread()
{
    color_sequence_pre = get_color();
    color_tran_count = 0;
    motor->SetSelectedSensorPosition(0, 0, 10);
    motor->ConfigMotionCruiseVelocity(spin_control_vel, 10);
    creddd = c_numb_serson_return = angle_to_enc((spin_numb + spin_numb_comp)*360);
    motor->Set(ControlMode::MotionMagic,c_numb_serson_return);   
    time_count[Spin] = 0; 
    is_finished_spin_control = false;
    spin_pos_error = 0;
    
    ddd = get_position_error(c_numb_serson_return,motor->GetSelectedSensorPosition());
    while (color_sequence_pre == 4)
    {
      color_sequence_pre = get_color();
      motor->Set(ControlMode::MotionMagic,c_numb_serson_return);
    }
    color_temp_last = color_sequence_pre;
    while (!isInterrupted())
    {
      /* code */
      COLOR color = get_color();
      spin_pos_error = get_position_error(c_numb_serson_return,motor->GetSelectedSensorPosition());
      if(color_is_changed(color))
        color_tran_count++;
      if(color_tran_count >= 4 * 2 * spin_numb)
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
        // curr_position = motor->GetSelectedSensorPosition();
        c_numb_serson_return = (angle_to_enc(45) + spin_numb_comp +c_numb_serson_return);
        motor->Set(ControlMode::MotionMagic,c_numb_serson_return);    
      }
      else
      {
        std::cout<<"spin_control"<<std::endl;
      }
      timer_sleep(0,reset_period);
      // usleep(reset_period);
    }
    interrupt();
    motor->Set(ControlMode::PercentOutput,0); 
    color_tran_count = 0;
}
///< 颜色传感器联系校验
// 注意: 使用前要先清上次的颜色
bool Dials::color_sequence_check(COLOR curr)
{
  return false;

}
///< 检测颜色是否变化
bool Dials::color_is_changed(COLOR curr)
{
  if(curr <4)
  {
    if(color_temp_last != curr)
    {
      color_temp_last = curr;
      return true;
    }
  }
  return false;
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
        return (abs(error)/(error))*color_angle;
    } 
    else
    {
        return error * color_angle * -1;
    }
    
}
///< 旋转转盘到指定颜色
void Dials::pos_control_thread()
{
    color_sequence_pre = get_color();
    motor->SetSelectedSensorPosition(0, 0, 10);
    while (color_sequence_pre == 4)
    {
      color_sequence_pre = get_color();
      motor->Set(ControlMode::MotionMagic,angle_to_enc(20));
    }   
    color_tran_count = 0;
    is_finished_pos_control = false;
    motor->SetSelectedSensorPosition(0, 0, 10);
    motor->ConfigMotionCruiseVelocity(spin_control_vel, 10);
    target_angle = optimal_path(target_color,color_sequence_pre);
    motor->Set(ControlMode::MotionMagic,angle_to_enc(target_angle));   
    time_count[Pos] = 0; 
    int color_numb = target_angle/color_angle;
    spin_pos_error = 0;
    while (!isInterrupted())
    {
      /* code */
      COLOR color = get_color();
      spin_pos_error = get_position_error(angle_to_enc(target_angle),motor->GetSelectedSensorPosition());
      if(color_is_changed(color))
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
              if(get_color() == target_color)
              {
                is_finished_pos_control = true;
                interrupt();
              }

            }
        }
      }
      else
      {
          motor->Set(ControlMode::MotionMagic,angle_to_enc(target_angle)); 
      }
      timer_sleep(0,reset_period);
      // usleep(reset_period);
    }
    interrupt();
}

///< 获取转盘旋转是否完成
bool Dials::spin_control_is_finished(void)
{
  return is_finished_spin_control;
}

///< 装盘初始化
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
    motor->ConfigMotionCruiseVelocity(300, 10);
    motor->ConfigMotionAcceleration(100, 10);

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
bool sww1 = false;
bool sww2 = false;
///< 升起
void Dials::lift()
{
    solenoid[0]->Set(true);
    solenoid[1]->Set(false); 
}
///< 收回
void Dials::put_down()
{
    solenoid[0]->Set(false);
    solenoid[1]->Set(true); 
}
///< 是否到达
bool Dials::is_arrived()
{
  return !(is_arrived_sw->Get());
}
///< 失能
void Dials::disable()
{
  interrupt();
  is_finished_spin_control = false;
  is_finished_pos_control = false;
  state = ALL_CTROL;
  if(!is_arrived())
  {
    put_down();
  }
}
///< 校准测试
void Dials::check_test_display()
{
    COLOR cur_color = get_color();
    frc::SmartDashboard::PutNumber("color",cur_color);
    frc::SmartDashboard::PutNumber("R", detectedColor.red);
    frc::SmartDashboard::PutNumber("G", detectedColor.green);
    frc::SmartDashboard::PutNumber("B", detectedColor.blue);
}
void Dials::check_test()
{
  float value[ALL_COLOR][3] = {0};
   std::string name[ALL_COLOR] = {"blue","green","red","yellow"};
  for(int i = 0;i<ALL_COLOR;i++)
  {
    wpi::StringRef name_tempr = name[i] + "R_value";
    wpi::StringRef name_tempg = name[i] + "G_value";
    wpi::StringRef name_tempb = name[i] + "B_value";
    
    value[i][0] = get_number(name_tempr,detectedColor.red,0.0,1.0);
    value[i][1] = get_number(name_tempg,detectedColor.green,0.0,1.0);
    value[i][2] = get_number(name_tempb,detectedColor.blue,0.0,1.0);
    color_target[i] = frc::Color(value[i][0], value[i][1], value[i][2]);
    m_colorMatcher.AddColorMatch(color_target[i]);
  }        
}
#ifdef DIALS_DEBUG
int temp_debug = 0;
void Dials::display()
{
    frc::SmartDashboard::PutNumber("dials_d:", dials_d);
    frc::SmartDashboard::PutNumber("color_angle:", color_angle);
    frc::SmartDashboard::PutNumber("curr_position", curr_position);
    frc::SmartDashboard::PutNumber("dials_perimeter", dials_perimeter);
    frc::SmartDashboard::PutNumber("arc_length", arc_length);
    frc::SmartDashboard::PutNumber("spin_numb_comp", spin_numb_comp);
    frc::SmartDashboard::PutNumber("frictiongear_d", frictiongear_d);
    frc::SmartDashboard::PutNumber("c_numb_serson_return", c_numb_serson_return);
    frc::SmartDashboard::PutNumber("is_finished_spin_control", is_finished_spin_control);
    frc::SmartDashboard::PutNumber("Red", detectedColor.red);
    frc::SmartDashboard::PutNumber("Green", detectedColor.green);
    frc::SmartDashboard::PutNumber("Blue", detectedColor.blue);
    frc::SmartDashboard::PutNumber("Confidence", confidence);
    frc::SmartDashboard::PutNumber("spin_pos_error", spin_pos_error);
    frc::SmartDashboard::PutNumber("is_finished_spin_pos_err", is_finished_spin_pos_err);
    frc::SmartDashboard::PutNumber("reduction_ratiop", reduction_ratiop);

    frc::SmartDashboard::PutNumber("ddd", ddd);
    frc::SmartDashboard::PutNumber("creddd", creddd);
    frc::SmartDashboard::PutNumber("color_tran_count", color_tran_count);


    frc::SmartDashboard::PutNumber("color_sequence_pre",color_sequence_pre);
    frc::SmartDashboard::PutNumber("abs(spin_pos_error)",abs(spin_pos_error));
    frc::SmartDashboard::PutNumber("target_angle",target_angle);
    frc::SmartDashboard::PutNumber("is_arrived",is_arrived());
    // frc::SmartDashboard::PutNumber("is_finished_spin_pos_err",is_finished_spin_pos_err);
  
    if(color_is_changed(cur_color))
      frc::SmartDashboard::PutNumber("filer_color", temp_debug++);

thread_debug();
    // frc::SmartDashboard::PutNumber("spin_control_thread_status", spin_control_thread_status);
}
// ///< 调试时设置参数
void Dials::set_para()
{
  double comp = frc::SmartDashboard::GetNumber("spin_numb_comp", spin_numb_comp);
  if((comp != spin_numb_comp)) { spin_numb_comp = comp; }

    sww1 = get_number("sww1",sww1,0,2);


    sww2 = get_number("sww2",sww2,0,2);
}

#endif

