#ifndef __STATUS_LED_H
#define __STATUS_LED_H
#include <frc/Solenoid.h>
#include <queue>
#include <frc/PowerDistributionPanel.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "BaseModule.h"
enum LAMP_MODE {U,A,ALL_MODE};
struct status_led
{
  int times = 0;
  bool status[ALL_MODE] = {false};
  int count = 0;
  int times_count = 0xFF;
  LAMP_MODE mode = ALL_MODE;
};
class Status_led
{
private:
  frc::Solenoid *solenoid[2];

  int times_buf = 200;
  int used_s = 0;
  bool run_status = false;

  std::queue<status_led> status_queue;
  status_led  run_led;
  // frc::PowerDistributionPanel pdp{0};
public:
  Status_led(){
    solenoid[0] = new frc::Solenoid(20,4);
    solenoid[1] = new frc::Solenoid(20,5);

  }

  ~Status_led(){
    delete solenoid[0];
    delete solenoid[1];
  }
frc::PowerDistributionPanel pdp{0};
///< 双灯同时闪烁
bool set_lamp_flicker_union(int times)
{
  status_led led;
  int times_temp = times;
  if(times_temp > times_buf)
    times_temp = times_buf;
  else if(times_temp < 0)
    times_temp = 0;
  led.times = times_temp;
  led.mode = U;
  led.count = 0;
  led.times_count = 0;
  led.status[U] = false;
  led.status[A] = false;
  status_queue.push(led);
}
///<双灯交替闪烁
bool set_lamp_flicker_alternate(int times)
{
  status_led led;
  int times_temp = times;
  if(times_temp > times_buf)
    times_temp = times_buf;
  else if(times_temp < 0)
    times_temp = 0;
  led.times = times_temp;
  led.mode = U;
  led.count = 0;
  led.times_count = 0;
  led.status[U] = false;
  led.status[A] = true;
  status_queue.push(led);
}
///< 灯状态更新
void updata_lamp()
{
  if(run_status == false && status_queue.empty() == false)
  {
    run_led = status_queue.front();
    status_queue.pop();
  }
  if(run_led.mode < ALL_MODE)
  {
    if(run_led.times_count < times_buf)
    {
      if(run_led.count < (21 - run_led.times))
      {
        run_led.count ++;
        // run_led.status[U]  = !run_led.status[U];
        // run_led.status[A]  = !run_led.status[A];
      }
      else
      {
        run_led.status[U]  = !run_led.status[U];
        run_led.status[A]  = !run_led.status[A];
        run_led.count = 0;
      }
      run_led.times_count++;
      run_status = true;
    }
    else
    {
      run_led.status[U] = false;
      run_led.status[A] = false;
      run_status = false;
    }
  }
  else{
      run_led.status[U] = false;
      run_led.status[A] = false;
  }
    solenoid[U]->Set(run_led.status[U]);
    solenoid[A]->Set(run_led.status[A]);
}
void test()
{
    bool get5 = frc::SmartDashboard::GetNumber("run_led.status[U]",run_led.status[U]);
    if(run_led.status[U] != get5)
      run_led.status[U] = get5;
    bool get6 = frc::SmartDashboard::GetNumber("run_led.status[A]",run_led.status[A]);
    if(run_led.status[A] != get6)
      run_led.status[A] = get6;

  solenoid[U]->Set(run_led.status[U]);
  solenoid[A]->Set(run_led.status[A]);
}
void test_dis()
{
      frc::SmartDashboard::PutNumber("run_led.status[U]",run_led.status[U]);
frc::SmartDashboard::PutNumber("run_led.status[A]",run_led.status[A]);
}
};

#endif