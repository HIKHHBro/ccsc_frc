#ifndef __RC_H
#define __RC_H
#include <frc/Joystick.h>
#include <frc/XboxController.h>
using namespace frc;
class RC
{
private:
   Joystick* joystick;
  //  XboxController* xboxController;
public:
  RC(int id);
  ~RC();
  // // bool GetABtn
  float getX();
  float getY();
  float getZ();

};



#endif