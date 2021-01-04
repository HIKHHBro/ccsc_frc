#ifndef __RC_H
#define __RC_H
#include <frc/Joystick.h>
#include <frc/XboxController.h>
using namespace frc;
class RC
{
private:
   Joystick* joystick;
   XboxController* xboxController;
public:
  RC(XboxController *xbox);
  RC(Joystick *joy);
  ~RC();
  // bool GetABtn

};


RC::~RC()
{
  // if(joystick != nullptr)
  //   delete joystick;
  // if(xboxController != nullptr)
  //   delete xboxController;
}

#endif