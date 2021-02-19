#include "rc.h"
#include "iostream"
#include <frc/smartdashboard/smartdashboard.h>
RC::RC(int id)
{
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
// float RC::getX()
// {
//   return (filter(joystick->GetRawAxis(0),0.2) * chassis_speed[0]);
// }
// float RC::getY()
// {
//   return -(filter(joystick->GetRawAxis(1),0.2)* chassis_speed[1]);
// }
// float RC::getZ()
// {
//   return filter(joystick->GetRawAxis(2),0.1) * chassis_speed[2];
// }


// bool RC::is_grab()
// {
//     // return xbox->GetBumper(frc::GenericHID::kLeftHand);
// }

// bool RC::is_reset()
// {
//   //  if(xbox->GetBackButton() && xbox->GetStartButton())
//   //  {
//   //   if(reset_count > unit_time)
//   //   {
//   //     return true;
//   //   }
//   //   else 
//   //   {
//   //      reset_count++;
//   //      return false;
//   //   }
//   //  }
//   //  else
//   //  {
//   //    reset_count = 0;
//   //     return false;
//   //  } 
// }
// bool RC::is_lift()
// {
//   // return xbox->GetBumper(frc::GenericHID::kRightHand);
// }
// // float get_angle()
// // {
// //   return ;
// // }
// bool RC::is_spin()
// {
//   return joystick->GetRawButton(11);
// }
// #endif

// #ifdef XBON_RC
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
  pitch_angle = (pitch_angle) > (23) ? (23) : (pitch_angle);
  pitch_angle = (pitch_angle) < (0) ? (0) : (pitch_angle);
  return pitch_angle;
#endif

#ifdef JOY_RC
  return (joystick->GetRawAxis(3)-1) * -11.5;
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
  return joystick->GetRawButton(7);
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
bool RC::is_used_auto_aim()
{
#ifdef XBON_RC
  if(xbox->GetYButtonPressed())
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
  if(joystick->GetRawButtonPressed(11))
  {
    used_auto_aim_flag = true;
  }
  if(joystick->GetRawButtonReleased(11) && used_auto_aim_flag)
  {
    used_auto_aim_flag = false;
    return true;
  }
  return false;
#endif
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
    // frc::SmartDashboard::PutNumber("1:", xbox->GetAButton());
    // frc::SmartDashboard::PutNumber("2:", xbox->GetBackButton());
    // frc::SmartDashboard::PutNumber("3", xbox->GetBButton());
    // frc::SmartDashboard::PutNumber("4", xbox->GetBumper(frc::GenericHID::kLeftHand ));
    // frc::SmartDashboard::PutNumber("6",xbox->GetRawButton(0));
    // frc::SmartDashboard::PutNumber("7",xbox->GetPOV(0));
    // frc::SmartDashboard::PutNumber("8",xbox->GetRawAxis(0));
    // frc::SmartDashboard::PutNumber("9",xbox->GetStartButton());
    // frc::SmartDashboard::PutNumber("10",xbox->GetStickButton(frc::GenericHID::kLeftHand));
    // frc::SmartDashboard::PutNumber("11",xbox->GetTriggerAxis(frc::GenericHID::kLeftHand));
    // frc::SmartDashboard::PutNumber("12",xbox->GetX(frc::GenericHID::kLeftHand));
    // frc::SmartDashboard::PutNumber("13",xbox->GetY(frc::GenericHID::kLeftHand));
    // frc::SmartDashboard::PutNumber("14",xbox->GetYButton());
    frc::SmartDashboard::PutNumber("reset",is_reset());
    frc::SmartDashboard::PutNumber("getPitch",getPitch());
#ifdef LIFT_DEBUG
    frc::SmartDashboard::PutNumber("is_lift",xbox->GetBumper(frc::GenericHID::kRightHand));
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