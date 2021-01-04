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
TalonFX *fx;
Joystick *_joystick;
XboxController * xbox;

// frc::BuiltInAccelerometer accelerometer;
// auto xAccelFilter = frc::LinearFilter<double>::MovingAverage(10);
int kTimeoutMs = 10;
void Robot::RobotInit() 
{
   chassis = new Chassis();
  //  chassis->set_series(0);
#if 1
  //  xbox = new XboxController(0);
  _joystick = new Joystick(0);
            // fx = new TalonFX(3);  
            // fx->ConfigFactoryDefault();
            // /* first choose the sensor */
            // fx->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 0);
            // // fx->SetSensorPhase(true);
            // fx->ConfigNeutralDeadband(0);
            // /* set the peak and nominal outputs */
            // fx->ConfigNominalOutputForward(0, 0);
            // fx->ConfigNominalOutputReverse(0, 0);
            // fx->ConfigPeakOutputForward(1, 0);
            // fx->ConfigPeakOutputReverse(-1, 0);
            // /* set closed loop gains in slot0 */
            // fx->Config_kF(0, 0.6, 0);
            // fx->Config_kP(0, 0.25, 0);
            // fx->Config_kI(0, 0.0, 0);
            // fx->Config_kD(0, 0.7, 0);
            // fx->ConfigClosedLoopPeriod(0,1,0);
            // fx->ConfigVelocityMeasurementPeriod(VelocityMeasPeriod::Period_100Ms,0);
            // fx->ConfigClosedloopRamp(0.5,0);


  // _joystick = new Joystick(1);
  // fx = new TalonFX(1);
	// 	fx->ConfigFactoryDefault();
  //       /* first choose the sensor */
	// 	fx->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, kTimeoutMs);
	// 	// fx->SetSensorPhase(true);

	// 	/* set the peak and nominal outputs */
	// 	fx->ConfigNominalOutputForward(0, kTimeoutMs);
	// 	fx->ConfigNominalOutputReverse(0, kTimeoutMs);
	// 	fx->ConfigPeakOutputForward(1, kTimeoutMs);
	// 	fx->ConfigPeakOutputReverse(-1, kTimeoutMs);
	// 	/* set closed loop gains in slot0 */
	// 	fx->Config_kF(0, 0.3, kTimeoutMs);
	// 	fx->Config_kP(0, 0.05, kTimeoutMs);
	// 	fx->Config_kI(0, 0.0, kTimeoutMs);
	// 	fx->Config_kD(0, 0.0, kTimeoutMs);



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
  // double filteredXAccel = xAccelFilter.Calculate(accelerometer.GetX());
  //  frc2::CommandScheduler::GetInstance().Run();
    // std::cout<<"陀螺仪是否在线:"<<chassis->check_gyro()<<std::endl;
   
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

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

}

/**
 * This function is called periodically during operator control.
 */
float target[3];
void Robot::TeleopPeriodic() 
{
    //TODO: 传入指令  -1~1
    target[0] = _joystick->GetRawAxis(0);
    target[1] =_joystick->GetRawAxis(1);
    target[2] =_joystick->GetRawAxis(2);
    if(abs(target[0])<0.1) target[0] = 0;
    if(abs(target[1])<0.1) target[1] = 0;
    if(abs(target[2])<0.1) target[2] = 0;
    std::cout<<"1 = "<<_joystick->GetRawAxis(0);
    std::cout<<"2 = "<<_joystick->GetRawAxis(1);
    std::cout<<"3 = "<<_joystick->GetRawAxis(2)<<std::endl;
  chassis->rc_run(target[0],target[1],target[2]);
}

/**
 * This function is called periodically during test mode.
 */
 double leftYstick ,rightYstick,vzYstick;
 double last_lefts;
void Robot::TestPeriodic() 
{
  //  leftYstick = _joystick->GetRawAxis(1)* 500.0 * 2048 / 600;
  //  rightYstick = _joystick->GetRawAxis(0)* 500.0 * 2048 / 600;
  //  vzYstick = _joystick->GetRawAxis(2)* 10;
  //  chassis->rc_run(leftYstick ,rightYstick,vzYstick);
  //  chassis->rc_run(1000,0,0);

  //  std::cout<< leftYstick * 500.0 * 2048 / 600<<std::endl;
  //  int tmp_v = fx->GetSelectedSensorVelocity();
  // //  fx->GetPIDConfigs(&pid_temp,0,10);
  //  std::cout << "v"<< tmp_v << std::endl;
  // fx->Set(ControlMode::Velocity,leftYstick * 500.0 * 2048 / 600);
  // int tmp_v = fx->GetSelectedSensorVelocity();
  // leftYstick = xbox->GetRawAxis(1)*500;
// chassis->motion_model(0,0,1000,0);
// if(leftYstick - last_lefts >30)
// {
//   leftYstick = 30 + last_lefts;
// }
// if((leftYstick - last_lefts < -30))
// {
//   leftYstick = last_lefts - 30;
// }
//  int  tmp_v = chassis->motor[Chassis::M3]->GetSelectedSensorPosition();
//  chassis->updata_series();
// int tmp_s = fx->GetSelectedSensorPosition();
  // std::cout<<"s"<<chassis->wheel_s[Chassis::M3]<<"v"<<tmp_v<<std::endl;
// if(xbox->GetAButton())
// {

//   // fx->Set(ControlMode::Velocity,leftYstick);
//   //  int temp = chassis->start_auto_run();
   
//   //  std::cout<<"temp"<<std::endl;
// }
// if(xbox->GetBButton())
// {
//   chassis->exit_auto_run();
//    std::cout<<"exit"<<std::endl;
// }
//   last_lefts = leftYstick;
// std::cout<<"text"<<"status<<"<<chassis->get_auto_run_status()<<std::endl;


}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
