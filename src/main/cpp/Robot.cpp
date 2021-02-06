/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>
#include <frc/BuiltInAccelerometer.h>
#include <frc/LinearFilter.h>
#include "ctre/Phoenix.h"
#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <frc/livewindow//LiveWindow.h>
#include "my_thread.h"


void Robot::RobotInit() 
{

  chassis = new Chassis(1);
  // dials = new Dials(9);
    // lifting = new Lifting(4);
  rc = new RC(0);
  shoot = new Shoot(0,7);
  grab = new Grab(5,20,0,0.02,0.01);

  // chassis->display();
  rc->display();
#ifdef GRAB_DEBUG
    grab->display();
#endif
#ifdef LIFT_DEBUG
  lifting->display();
#endif  

#ifdef SHOOT_DEBUG
  shoot->display();
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

void Robot::RobotPeriodic() 
{
  
#ifdef COM_DEBUG
    float Get1 = frc::SmartDashboard::GetNumber("sepp[0]",sepp[0]);
    if(sepp[0] != Get1){sepp[0] = Get1;}

    float Get2 = frc::SmartDashboard::GetNumber("sepp[1]",sepp[1]);
    if(sepp[1] != Get2){sepp[1] = Get2;}

    float Get3 = frc::SmartDashboard::GetNumber("sepp[2]",sepp[2]);
    if(sepp[2] != Get3){sepp[2] = Get3;}
    chassis->rc_run(sepp[0],sepp[1],sepp[2]);
    chassis->debug();
    rc->debug();

#endif

#ifdef CHASSIS_DEBUG
  chassis->debug();
#endif
#ifdef GRAB_DEBUG
  grab->debug();
#endif
#ifdef LIFT_DEBUG
  lifting->debug();
#endif

#ifdef SHOOT_DEBUG
  shoot->debug();
#endif

  rc->debug();
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
  
}

void Robot::AutonomousPeriodic() {   

  // chassis->start_auto_run();
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
}

/**
 * This function is called periodically during operator control.
 */

void Robot::TeleopPeriodic() 
{
  /* 抓取*/
  if(rc->is_grab())
  {
    shoot->open_horizontal_transfer();
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
  chassis->rc_run(rc->getX(),rc->getY(),rc->getZ());
  /* 云台pitch */
  shoot->set_gimbal_angle(rc->getPitch());


  /* 复位 */
  if(rc->is_reset())
  {
    shoot->start_join();
  }
  else
  {
    shoot->interrupt();
  }
}

/**
 * This function is called periodically during test mode.
 */
// RC rc(0);
void Robot::TestInit()
{

  // std::cout<<"按键"<<rc->is_spin()<<std::endl;
  

}
void Robot::TestPeriodic() 
{

  // std::cout<<"按键"<<rc->is_spin()<<std::endl;
  // dials->get_color();
  // dials->display();
  // if(rc->is_spin())
  // {
  //   dials->start_spin_control(1);
  // }
  // else{
  //   dials->interrupt();
  // }
  // chassis->angle_control(chassis->test_angle);
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
