#include "rc.h"

RC::RC(int id)
{
  joystick = new Joystick(id);
}

RC::~RC()
{
  // if(joystick != nullptr)
  //   delete joystick;
  // if(xboxController != nullptr)
  //   delete xboxController;
}

float RC::getX()
{
  return joystick->GetRawAxis(0);
}
float RC::getY()
{
  return joystick->GetRawAxis(0); 
}
float RC::getZ()
{

}

