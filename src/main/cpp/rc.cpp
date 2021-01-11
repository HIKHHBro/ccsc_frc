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
#ifdef JOY_RC
float RC::getX()
{
  return joystick->GetRawAxis(0);
}
float RC::getY()
{
  return joystick->GetRawAxis(1); 
}
float RC::getZ()
{
    return joystick->GetRawAxis(2); 

}

#endif

#ifdef XBON_RC
float RC::getX()
{
  return xbox->GetRawAxis(0);
}
float RC::getY()
{
  return xbox->GetRawAxis(1); 
}
float RC::getZ()
{
    return xbox->GetRawAxis(2); 
}
bool RC::is_grab()
{
    return xbox->GetBumper(frc::GenericHID::kLeftHand);
}

bool RC::is_reset()
{
   if(xbox->GetBackButton() && xbox->GetStartButton())
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
  return xbox->GetBumper(frc::GenericHID::kRightHand);
}
// float get_angle()
// {
//   return ;
// }
#endif
#ifdef RC_DEBGU

void RC::display()
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
#ifdef LIFT_DEBUG
    frc::SmartDashboard::PutNumber("is_lift",xbox->GetBumper(frc::GenericHID::kRightHand));
#endif



}
#endif