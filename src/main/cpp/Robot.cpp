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


float sepp[3];
void Robot::RobotInit() 
{
  chassis = new Chassis(0);
  dials = new Dials(3);
  grab = new Grab(0,20,0,0.02,0.01);
  rc = new RC(0);
  lifting = new Lifting(5);
  chassis->display();
  rc->display();
    // frc::SmartDashboard::PutNumber("sepp[0]",sepp[0]);
    // frc::SmartDashboard::PutNumber("sepp[1]",sepp[1]);
    // frc::SmartDashboard::PutNumber("sepp[2]",sepp[2]);


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
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {

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
  std::cout <<( m_autonomousCommand ==nullptr) << std::endl;
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


  // lifting->display();
#ifdef GRAB_DEBUG
    grab->display();
#endif
#ifdef LIFT_DEBUG
  lifting->display();
#endif

}

/**
 * This function is called periodically during operator control.
 */
float target[3];
void Robot::TeleopPeriodic() 
{
    // //TODO: 传入指令  -1~1
    // target[0] = _joystick->GetRawAxis(0);
    // target[1] =_joystick->GetRawAxis(1);
    // target[2] =_joystick->GetRawAxis(2);
    // if(abs(target[0])<0.1) target[0] = 0;
    // if(abs(target[1])<0.1) target[1] = 0;
    // if(abs(target[2])<0.1) target[2] = 0;
    // std::cout<<"1 = "<<_joystick->GetRawAxis(0);
    // std::cout<<"2 = "<<_joystick->GetRawAxis(1);
    // std::cout<<"3 = "<<_joystick->GetRawAxis(2)<<std::endl;
    // chassis->rc_run(target[0],target[1],target[2]);
    // chassis->milemter();
    //   if(rc->is_grab())
    //   grab->put_down();
    // else
    // {
    //   grab->put_up();
    // }
    // if (rc->is_reset())
    // {
    //   if(!lifting->reset_once)
    //   {
    //     lifting->reset_once = true;//防止复位失败后一直从进复位线程
    //     lifting->reset();
    //   }
    // }
    // else
    // {
    //   lifting->reset_once = false;
    //   lifting->interrupt();
    // }
    // if(rc->is_lift())
    //   lifting->lift();
    // else
    // {
    //   lifting->disable_motor();
    // }
    
  // grab->debug();
  // lifting->debug();
  // rc ->display();
  chassis->rc_run(rc->getX(),rc->getY(),rc->getZ());
#ifdef GRAB_DEBUG
  grab->debug();
#endif
#ifdef LIFT_DEBUG
  lifting->debug();
  rc->debug();
#endif
}

/**
 * This function is called periodically during test mode.
 */
// RC rc(0);
void Robot::TestInit()
{



}
void Robot::TestPeriodic() 
{
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
