#ifndef __RC_H
#define __RC_H
#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include "BaseModule.h"
using namespace frc;
class RC
{
private:
#ifdef JOY_RC
   Joystick* joystick;
#endif
#ifdef XBON_RC
   XboxController* xbox;
#endif
  
public:
  RC(int id);
  ~RC();
  float getX();
  float getY();
  float getZ();
  bool is_grab();
  void display();

};



#endif