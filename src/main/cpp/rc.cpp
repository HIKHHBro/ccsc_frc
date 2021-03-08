#include "rc.h"
#include "iostream"
#include <frc/smartdashboard/smartdashboard.h>
RC::RC(int id,float max_angle = 23)
{
  pitch_max_angle = max_angle;
#ifdef JOY_RC
   joystick = new Joystick(id);
#endif
#ifdef XBON_RC
   xbox = new XboxController(id);
#endif
  
}

RC::~RC()
{
#ifdef JOY_RC
   delete joystick;
#endif
#ifdef XBON_RC
   delete xbox;
#endif
}
///< 过滤中间区间，防止误触
float RC::filter(float data,float section)
{
    float d = data;
    if(d > 0)
    {
        if(d >0.2)
          d = (d-0.2);//TODO: 待确认猎鹰做大速度
          else d =0;
    }
    else if(d < 0)
    {
        if(d <-0.2)
          d = (d+0.2);
        else d =0;
    }
    return d;
}

#ifdef JOY_RC
#endif
///< x方向速度 0 ~ 5000 mm/s 
float RC::getX()
{
#ifdef XBON_RC
  return (filter(xbox->GetRawAxis(0),0.2) * chassis_speed[0]);
#endif

#ifdef JOY_RC
  return (filter(joystick->GetRawAxis(0),0.2) * chassis_speed[0]);
#endif

}
///< y方向速度 0 ~ 5000 mm/s 
float RC::getY()
{
#ifdef XBON_RC
  return (filter(xbox->GetRawAxis(1),0.2)* chassis_speed[1] * -1);
#endif

#ifdef JOY_RC
  return (filter(joystick->GetRawAxis(1),0.2) * chassis_speed[1] * -1);
#endif
}
///< 角速度方向速度 0 ~ 4 rad/s  
float RC::getZ()
{
#ifdef XBON_RC
    return filter(xbox->GetRawAxis(4),0.1) * chassis_speed[2];
#endif

#ifdef JOY_RC
  return filter(joystick->GetRawAxis(2),0.1) * chassis_speed[2];
#endif
}
///< 云台角度
//TODO: 待测试值
float RC::getPitch()
{
#ifdef XBON_RC
  pitch_angle -= xbox->GetRawAxis(5);
  pitch_angle = (pitch_angle) > (pitch_max_angle) ? (pitch_max_angle) : (pitch_angle);
  pitch_angle = (pitch_angle) < (0) ? (0) : (pitch_angle);
  return pitch_angle;
#endif

#ifdef JOY_RC
  pitch_angle = (joystick->GetRawAxis(3)-1) * -(pitch_max_angle/2.0);
  return pitch_angle;
#endif
}
bool RC::is_grab()
{
#ifdef XBON_RC
    return xbox->GetBumper(frc::GenericHID::kRightHand);
#endif

#ifdef JOY_RC
  return joystick->GetRawButton(2);  
#endif
}

bool RC::is_reset()
{
#ifdef XBON_RC
   if(xbox->GetBackButton() && xbox->GetStartButton())
#endif
#ifdef JOY_RC
  if(joystick->GetRawButton(5) && joystick->GetRawButton(6))
#endif
   {
    if(reset_count > unit_time)
    {
      return true;
    }
    else 
    {
       reset_count++;
       return false;
    }
   }
   else
   {
     reset_count = 0;
      return false;
   } 


}
bool RC::is_lift()
{
#ifdef XBON_RC
  return xbox->GetBumper(frc::GenericHID::kRightHand);
#endif

#ifdef JOY_RC
  
  return joystick->GetRawButton(8);
#endif
}
///< 高挂机构完全升出
bool RC::is_reach_out()
{
#ifdef XBON_RC
  return false;
#endif

#ifdef JOY_RC
  if(joystick->GetRawButtonPressed(7))
  {
    reach_out_flag = true;
    if(!reach_out_flag_debug)
      lift_flag = false;
  }
  if(joystick->GetRawButtonReleased(7) && reach_out_flag)
  {
    reach_out_flag = false;
    reach_out_flag_debug = !reach_out_flag_debug;
    // return reach_out_flag_debug;
  }
  return reach_out_flag_debug;
#endif
}
bool RC::is_dials_lift()
{
#ifdef XBON_RC
  return xbox->GetBumper(frc::GenericHID::kLeftHand);
#endif

#ifdef JOY_RC
  return joystick->GetRawButton(3);
#endif
}
bool RC::is_spin()
{
#ifdef XBON_RC
  return xbox->GetAButton();
#endif

#ifdef JOY_RC
  return joystick->GetRawButton(9);
#endif
}
bool RC::is_pos()
{
#ifdef XBON_RC
  return xbox->GetXButton();
#endif

#ifdef JOY_RC
  return joystick->GetRawButton(10);
#endif
}
bool RC::is_shoot()
{
#ifdef XBON_RC
  if(abs(xbox->GetRawAxis(3)) ==1)
  {
    return true;
  }
  else
    return false;
#endif

#ifdef JOY_RC
  return joystick->GetRawButton(1);
#endif
}
//TODO: 测试自动状态切换
bool RC::is_used_auto_aim()
{
#ifdef XBON_RC
  if(xbox->GetYButtonPressed())auto
  {
    used_auto_aim_flag = true;
  }
  if(xbox->GetYButtonReleased() && used_auto_aim_flag)
  {
    used_auto_aim_flag = false;
    return true;
  }
  return false;
#endif

#ifdef JOY_RC
  // if(joystick->GetRawButtonPressed(11))
  // {
  //   used_auto_aim_flag = true;
  // }
  // if(joystick->GetRawButtonReleased(11) && used_auto_aim_flag)
  // {
  //   used_auto_aim_flag = false;
  //   used_auto_aim_flag_debug = !used_auto_aim_flag_debug;
  // }
  // return used_auto_aim_flag_debug;
  return joystick->GetRawButton(11);
#endif
}


void RC::clear_pitch()
{
  pitch_angle = 0;
}

#ifdef RC_DEBGU

void RC::display()
{
#ifdef CHASSIS_DEBUG
    frc::SmartDashboard::PutNumber("遥控给定X轴最大速度mm/s",chassis_speed[0]);
    frc::SmartDashboard::PutNumber("遥控给定Y轴最大速度mm/s",chassis_speed[1]);
    frc::SmartDashboard::PutNumber("遥控给定角速度rad/s",chassis_speed[2]);
#endif
}
void RC::debug()
{
    frc::SmartDashboard::PutNumber("reset",is_reset());
    if(used_auto_aim_flag_debug)
      frc::SmartDashboard::PutString("自瞄","开");
    else frc::SmartDashboard::PutString("自瞄","关");
    frc::SmartDashboard::PutNumber("pitch_angle",pitch_angle);
#ifdef LIFT_DEBUG
  frc::SmartDashboard::PutNumber("lift_flag",lift_flag);
  frc::SmartDashboard::PutNumber("reach_out_flag_debug",reach_out_flag_debug);

#endif

#ifdef CHASSIS_DEBUG
    float Get1 = frc::SmartDashboard::GetNumber("遥控给定X轴最大速度mm/s",chassis_speed[0]);
    if(chassis_speed[0] != Get1){chassis_speed[0] = Get1;}

    float Get2 = frc::SmartDashboard::GetNumber("遥控给定Y轴最大速度mm/s",chassis_speed[1]);
    if(chassis_speed[1] != Get2){chassis_speed[1] = Get2;}

    float Get3 = frc::SmartDashboard::GetNumber("遥控给定角速度rad/s",chassis_speed[2]);
    if(chassis_speed[2] != Get3){chassis_speed[2] = Get3;}

    frc::SmartDashboard::PutNumber("遥控X轴",getX());
    frc::SmartDashboard::PutNumber("遥控Y轴",getY());
    frc::SmartDashboard::PutNumber("遥控Z轴",getZ());

#endif
}

#endif