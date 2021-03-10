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
   float unit_time = 20;
   float chassis_speed[3] = {1000,1000,2};
   float pitch_angle = 0;
   bool used_auto_aim_flag = false;
   bool used_auto_aim_flag_debug = false;
   bool reach_out_flag = false;
   bool reach_out_flag_debug = false;

   
   bool lift_flag_debug = false;

   float pitch_max_angle = 23;
public:
  RC(int id,float max_angle);
  ~RC();
  float getX();
  float getY();
  float getZ();
  float getPitch();
  bool is_grab();
  void display();
  void debug();
  bool is_reset();
  bool is_lift();
  bool is_reach_out();
  float filter(float data,float section);
  bool is_spin();
  bool is_shoot();
  bool is_pos();
  bool changed_spin = true;
  bool changed_pos = true;
  bool is_dials_lift();
  bool is_used_auto_aim();
  void clear_pitch();
  bool lift_flag = false;

};



#endif