/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include "networktables/NetworkTableInstance.h"
#include <frc2/command/CommandScheduler.h>
#include <frc/BuiltInAccelerometer.h>
#include <frc/LinearFilter.h>
#include "ctre/Phoenix.h"
#include <frc/Joystick.h>
#include <frc/XboxController.h>

bool auto_flag = false;
void Robot::RobotInit() 
{

  chassis = new Chassis(1);
  dials = new Dials(8);
  lifting = new Lifting(5);
  rc = new RC(0,23);
  shoot = new Shoot(0,7);
  grab = new Grab(5,20,0,0.02,0.01);
  sleep(1);

#ifdef RC_DEBGU
  rc->display();
#endif
#ifdef GRAB_DEBUG
    grab->display();
#endif
#ifdef LIFT_DEBUG
  // lifting->display();
#endif  

#ifdef SHOOT_DEBUG
  shoot->display();
#endif

#ifdef DIALS_DEBUG
  dials->display();
#endif
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
bool faf=false;
void Robot::RobotPeriodic() 
{
  
#ifdef CHASSIS_DEBUG
  chassis->debug();
#endif
#ifdef GRAB_DEBUG
  grab->debug();
#endif
#ifdef LIFT_DEBUG
  // lifting->debug();
  // lifting->display();
#endif

#ifdef SHOOT_DEBUG
  shoot->debug();
#endif
#ifdef RC_DEBGU
  rc->debug();
#endif
#ifdef DIALS_DEBUG
  dials->display();
  dials->set_para();
#endif
  status_lamp.low_battery_tip();
  if(!shoot->get_reset_status() || !lifting->get_reset_status())
    status_lamp.set_tip_mode(Status_led::NO_Reset);
frc::SmartDashboard::PutNumber("ty",chassis->limelight->getTargetY());
  if(auto_flag)
  {
    status_lamp.set_tip_mode(Status_led::OPEN);
  }
  else{
    // status_lamp.set_tip_mode(Status_led::CLOSE);
  }
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {

  /* 底盘清零 */
  chassis->clear();
  shoot->interrupt();
  lifting->interrupt();
  dials->disable();
}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Schedule();
  }
  if(!shoot->get_reset_status())
    shoot->start_join();
  shoot->set_gimbal_angle(15);
  chassis->set_auto_point(1);
  chassis->start_auto_run();
  shoot->start_shoot();
  shoot->close_horizontal_transfer();
  grab->put_down();
  status_lamp.set_tip_mode(Status_led::OPEN);
  for(int i = 0;i<4;i++)
  {
    chassis->motor_pid[i]->PIDTuningsSet(0.3,10,0);
    // frc::SmartDashboard::PutString("auti","init auto");
  }
  auto_flag = true;
    
  
}
void Robot::AutonomousPeriodic() { 
  status_lamp.set_tip_mode(Status_led::OPEN);
  if(chassis->is_arrived_point())
  {
    switch (chassis->get_auto_point())
    {
    case 1:
      if(shoot->auto_shoot())
      {
        chassis->set_auto_point(2);
        chassis->set_map_point();
      }
      break;
    case 2:
      chassis->set_auto_point(3);
      break;
    case 3:
      if(shoot->auto_shoot())
      {
        chassis->set_auto_point(0);
      }
      break;
    default:
      chassis->set_auto_point(0);
      shoot->stop_shoot();
      shoot->close_horizontal_transfer();
      shoot->close_vertical_transfer();
      grab->put_up();

      break;
    }
  }
  else
  {
    if(chassis->get_auto_point() >1)
    {
      shoot->stop_vertical_transfer();
      shoot->open_horizontal_transfer();
    }
    else
    {
      shoot->close_horizontal_transfer();
      shoot->stop_vertical_transfer(); 
    }

  }
}

void Robot::TeleopInit() {
  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.
  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Cancel();
    m_autonomousCommand = nullptr;
  }
  for(int i = 0;i<4;i++)
  {
    chassis->motor_pid[i]->PIDTuningsSet(0.35,10,0);
     frc::SmartDashboard::PutString("telo","init telo");
  }
}

/**
 * This function is called periodically during operator control.
 */
int test_color = 0;
bool changed_lid = false;
void Robot::TeleopPeriodic() 
{
  grab->enable_compressor();
/* 抓取*/
  if(rc->is_grab())
  {
    // shoot->open_horizontal_transfer();
    shoot->stop_vertical_transfer();
    grab->put_down();
  }
  else
  {
    grab->put_up();
  }
/* 发射 */
  if(rc->is_shoot())
  {
    shoot->start_shoot();
  }
  else
  {
    shoot->stop_shoot();
  }
/* 如果没有发射或者抓球，关闭传送 */
  if((!rc->is_grab()) && (!rc->is_shoot()))
  {
    shoot->close_horizontal_transfer();
    shoot->close_vertical_transfer();
  }
/* 底盘 */
float auto_angle = chassis->limelight->getTargetX() / 30.0;
float auto_anglpitch = shoot->get_pitch_angle();
 frc::SmartDashboard::PutNumber("auto_angle",auto_angle);
 frc::SmartDashboard::PutNumber("auto_anglpitch",auto_anglpitch);
if(rc->is_used_auto_aim() && (abs(auto_angle) <0.5))
{
  LIMIT(auto_angle,-0.2,0.25);
  chassis->rc_run(rc->getX(),rc->getY(),auto_angle);
}
else
{
  chassis->rc_run(rc->getX(),rc->getY(),rc->getZ());
}
/* 云台pitch */
  if(rc->is_used_auto_aim())
  {
    status_lamp.set_tip_mode(Status_led::OPEN);
    shoot->set_gimbal_angle(auto_anglpitch);
  }
  else
  {
    // status_lamp.set_tip_mode(Status_led::CLOSE);
    shoot->set_gimbal_angle(rc->getPitch());
  }

/* 复位 */
  if(rc->is_reset())
  {
    shoot->start_join();
    lifting->start_join();
    rc->clear_pitch();
  }
  else
  {
    lifting->interrupt();
    shoot->interrupt();
  }
/* 转盘 */
  dials->get_color();
  if(rc->is_dials_lift())
  {
    dials->lift();
    if(rc->is_spin())
    {
      if(rc->changed_spin)
        dials->start_spin_control(3);
      rc->changed_spin = false;
    }
    else if(rc->is_pos())
    {
      if(rc->changed_pos)
        dials->start_pos_control((Dials::COLOR)test_color);
      rc->changed_pos = false;
    }
  }
  else
  {
    rc->changed_pos = true;
    rc->changed_spin = true;
    dials->disable();
  }
  if(rc->is_lift())
  {
    lifting->lift();
    rc->lift_flag = true;
  }
  else if(rc->is_reach_out() && !rc->lift_flag)
  {
    lifting->stretch_out();
  }
  else if(!rc->lift_flag)
  {
    lifting->shrink();
  }
// limelight->test_ultrasonic();
// limelight->get_camtran();
// status_lamp.set_tip_mode(Status_led::OPEN);

}

void Robot::TestInit()
{
  

}

void Robot::TestPeriodic() 
{
  // dials->check_test();
  shoot->set_test();
  dials->check_test_display();
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
