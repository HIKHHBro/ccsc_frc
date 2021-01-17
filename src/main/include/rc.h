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

   int reset_count = 0;
   float unit_time = 50;
   float chassis_speed[3] = {1000,1000,2};
public:
  RC(int id);
  ~RC();
  float getX();
  float getY();
  float getZ();
  bool is_grab();
  void display();
  void debug();
  bool is_reset();
  bool is_lift();
  float filter(float data,float section);
};



#endif