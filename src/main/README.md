/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#pragma once
#include "BaseModule.h"
#include "AHRS.h"

using namespace frc;
class Chassis 
{
 public:
    Chassis();
    ~Chassis();
    enum MotionModel {
        OMNIDIR4 =2, 
        STEER = 4};
    enum MOTOR {M1,M2,M3,M4};
    enum TARGET {x,y,z};
    float speed[4];
    float  wheel_s[4];
    double motion[3];
    void motion_model(int csys,float vx,float vy,float vz);
    float theta_m;//TODO: 闁跨喐鏋婚幏宄板絿闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔活潡鐟欐帟顔愰幏锟�
    bool check_gyro();
    void get_gyro();
    void get_run_target(float* target);
    void update_rc_data();
    void rc_run();
    bool milemter();
    void series_to_cm (int16_t  wheel);
    bool Chassis::to_position(float x,float y,float w);
    float wheel_theta = 45;
    float chassis_r = 382.835;//mm
    float wheel_r =  76.2闁挎冻鎷�
    float  motor_series = 0;
    
 private:

    float angle_to_radian = 0.01745f;//闁跨喕顫楃拋瑙勫鏉烆剟鏁撻弬銈嗗闁跨喐鏋婚幏锟�
    float auto_angle = 45
    AHRS *ahrs;
};
# 绾兛娆�
FRC Game Tools閿涳拷


## 閺冨爼妫块悙锟�
* [ ] *2020/12/26*
    * [x] 瀹搞儳鈻煎铏圭彌
    * [ ] 鎼存洜娲忕粙瀣碍濡楀棙鐏︾紓鏍у晸
        * [x] 閸忋劌鎮滄潪顔兼磽鏉烆喛绻嶉崝銊δ侀崹锟�
        * [ ] 鎼存洜娲忛柌宀€鈻肩拋锟�
        * [ ] 鎼存洜娲忕粚娲？娴肩儤婀�
    * [ ] 閻㈠灚婧€閿涘矂妾ч摶杞板崕
    * [ ] 閹貉冨煑PID 閸欘垶鈧喎瀹虫担宥囩枂閾诲秴鎮庨幒褍鍩�
* [ ] *2020/12/30*
    * [ ] 娴滄垵褰寸粙瀣碍濡楀棙鐏︾紓鏍у晸
    * [ ] 鎼存洜娲忔潻鎰З濡€崇€风拫鍐槸
    * [ ] 鎼存洜娲忛柌宀€鈻肩拋陇鐨熺拠锟�
    




## 鎼存洜娲忕拠瀛樻 
1. 鎼存洜娲忕敮鍐ㄧ湰閸欏﹥婧€娴ｆ挸娼楅弽鍥╅兇     
       
               y鏉烇拷           
               ^             
        2 [----|----] 1
            \  |  /  
              \|/_________ x鏉烇拷
              / \   
            /     \   
        3 [---------] 4   
   * 閸ф劖鐖ｇ化锟�:閸欒櫕澧滈崸鎰垼缁拷
   * 閺傜懓鎮�:闁棙妞傞柦鍫滆礋濮濓拷
闂€鍨閸楁洑缍�  濮ｎ偆鑳�
鐟欐帒瀹抽崡鏇氱秴  鎼达拷
閺冨爼妫块崡鏇氱秴  濮ｎ偆顫�

## 鎼达拷
navx(imu) https://www.kauailabs.com/dist/frc/2020/navx_frc.json  
Phoenix(閻氬酣鎷遍悽鍨簚) http://devsite.ctr-electronics.com/maven/release/com/ctre/phoenix/Phoenix-latest.json



## 閸э拷
鎼存挻婀佸▽鈥愁嚤閸忋儲鍨氶崝锟� 閺屻儳婀卾endordeps閺傚洣娆㈡径閫涚瑓闂堛垺婀侀弮鐘垫祲鐎电懓绨�.jsion閺傚洣娆� 

## 閺傚洦銆�
閻氬酣鎷遍崪瀛睵X :https://phoenix-documentation.readthedocs.io/en/latest/ch06_PrepRobot.html
navx(imu): https://pdocs.kauailabs.com/navx-mxp/software/roborio-libraries/c/



## 閻ゆ垹鍋�
vscode 閻ㄥ嚧ioLog闂団偓鐟曚椒绮堟稊鍫滅贩鐠ф牕瀵橀幍宥堝厴閻拷
閺嗗倹妞傜憴锝呭枀:鐎瑰顥婃禍鍞抙oenix 閸滃avx鐏忚精鍏橀悽銊ょ啊



### fx motion magic
璺矾璺�
/**
 * Phoenix Software License Agreement
 *
 * Copyright (C) Cross The Road Electronics.  All rights
 * reserved.
 * 
 * Cross The Road Electronics (CTRE) licenses to you the right to 
 * use, publish, and distribute copies of CRF (Cross The Road) firmware files (*.crf) and 
 * Phoenix Software API Libraries ONLY when in use with CTR Electronics hardware products
 * as well as the FRC roboRIO when in use in FRC Competition.
 * 
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * CROSS THE ROAD ELECTRONICS BE LIABLE FOR ANY INCIDENTAL, SPECIAL, 
 * INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE
 */

/**
 * Description:
 * The MotionMagic example demonstrates the motion magic control mode.
 * Tested with Logitech F710 USB Gamepad inserted into Driver Station.
 * 
 * Be sure to select the correct feedback sensor using configSelectedFeedbackSensor() below.
 *
 * After deploying/debugging this to your RIO, first use the left Y-stick 
 * to throttle the Talon manually. This will confirm your hardware setup/sensors
 * and will allow you to take initial measurements.
 * 
 * Be sure to confirm that when the Talon is driving forward (green) the 
 * position sensor is moving in a positive direction. If this is not the 
 * cause, flip the boolean input to the setSensorPhase() call below.
 * 
 * Controls:
 * Button 1: When held, put Talon in Motion Magic mode and allow Talon to drive [-10, 10] 
 * 	rotations.
 * Button 2: When pushed, the selected feedback sensor gets zero'd
 * Button 5(Left shoulder): When pushed, will decrement the smoothing of the motion magic down to 0
 * Button 6(Right shoulder): When pushed, will increment the smoothing of the motion magic up to 8
 * Left Joystick Y-Axis:
 * 	+ Percent Output: Throttle Talon FX forward and reverse, use to confirm hardware setup.
 * Right Joystick Y-Axis:
 * 	+ Motion Maigic: Servo Talon FX forward and reverse, [-10, 10] rotations.
 * 
 * Gains for Motion Magic
 * 
 * Supported Version:
 * - Talon FX: 20.2.3.0
 */
#include "Robot.h"
#include <sstream>

void Robot::RobotInit() {
    _talon = new TalonFX(3);
    _joy = new frc::Joystick(0);

    /* Factory default hardware to prevent unexpected behavior */
    _talon->ConfigFactoryDefault();

    /* Configure Sensor Source for Pirmary PID */
    _talon->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor,
                                        0, 
                                        10);

    /*
     * Talon FX does not need sensor phase set for its integrated sensor
     * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
     * and the user calls getSelectedSensor* to get the sensor's position/velocity.
     * 
     * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
     */
    //_talon->SetSensorPhase(false);
    _talon->SetInverted(TalonFXInvertType::CounterClockwise);

    /* Set relevant frame periods to be at least as fast as periodic rate */
    _talon->SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
    _talon->SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);

    /* Set the peak and nominal outputs */
    _talon->ConfigNominalOutputForward(0, 10);
    _talon->ConfigNominalOutputReverse(0, 10);
    _talon->ConfigPeakOutputForward(1, 10);
    _talon->ConfigPeakOutputReverse(-1, 10);

    /* Set Motion Magic gains in slot0 - see documentation */
    _talon->SelectProfileSlot(0, 0);
    _talon->Config_kF(0, 0.3, 10);
    _talon->Config_kP(0, 0.1, 10);
    _talon->Config_kI(0, 0.0, 10);
    _talon->Config_kD(0, 0.0, 10);

    /* Set acceleration and vcruise velocity - see documentation */
    _talon->ConfigMotionCruiseVelocity(1500, 10);
    _talon->ConfigMotionAcceleration(1500, 10);

    /* Zero the sensor */
    _talon->SetSelectedSensorPosition(0, 0, 10);
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {
    /* Get gamepad axis - forward stick is positive */
    double leftYstick = -1.0 * _joy->GetY();
    double RightYstick = -1.0 * _joy->GetY();
    if (fabs(leftYstick) < 0.10) { leftYstick = 0;} /* deadband 10% */
    if (fabs(RightYstick) < 0.10) { RightYstick = 0;} /* deadband 10% */

    /* Get current Talon FX motor output */
    double motorOutput = _talon->GetMotorOutputPercent();
    std::stringstream sb;
    /* Prepare line to print */
    sb << "\tOut%:" << motorOutput;
    sb << "\tVel:" << _talon->GetSelectedSensorVelocity(0);

    if(_joy->GetRawButton(2))
    {
        /* Zero the sensor */
        _talon->SetSelectedSensorPosition(0, 0, 10);
    }

    /**
     * Peform Motion Magic when Button 1 is held,
     * else run Percent Output, which can be used to confirm hardware setup.
     */
    if (_joy->GetRawButton(1)) {
        /* Motion Magic */ 
        
        /*2048 ticks/rev * 10 Rotations in either direction */
        double targetPos = RightYstick * 2048 * 10.0;
        _talon->Set(ControlMode::MotionMagic, targetPos);

        /* Append more signals to print when in speed mode */
        sb << "\terr:" << _talon->GetClosedLoopError(0);
        sb << "\ttrg:" << targetPos;
    } else {
        /* Percent Output */

        _talon->Set(ControlMode::PercentOutput, leftYstick);
    }

    if(_joy->GetRawButtonPressed(6))
    {
        /* Increase smoothing */
        ++_smoothing;
        if(_smoothing > 8) _smoothing = 8;
        std::cout << "Smoothing is set to: " << _smoothing << std::endl;
        _talon->ConfigMotionSCurveStrength(_smoothing, 0);
    }
    if(_joy->GetRawButtonPressed(5))
    {
        /* Decreasing smoothing */
        --_smoothing;
        if(_smoothing < 0) _smoothing = 0;
        std::cout << "Smoothing is set to: " << _smoothing << std::endl;
        _talon->ConfigMotionSCurveStrength(_smoothing, 0);
    }
    

    /* Instrumentation */
    Instrum::Process(_talon, &sb);
}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

璺矾璺�



##  闁喎瀹�
```
/**
 * Phoenix Software License Agreement
 *
 * Copyright (C) Cross The Road Electronics.  All rights
 * reserved.
 * 
 * Cross The Road Electronics (CTRE) licenses to you the right to 
 * use, publish, and distribute copies of CRF (Cross The Road) firmware files (*.crf) and 
 * Phoenix Software API Libraries ONLY when in use with CTR Electronics hardware products
 * as well as the FRC roboRIO when in use in FRC Competition.
 * 
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * CROSS THE ROAD ELECTRONICS BE LIABLE FOR ANY INCIDENTAL, SPECIAL, 
 * INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE
 */
/**
 * Example demonstrating the velocity closed-loop servo.
 * Tested with Logitech F350 USB Gamepad inserted into Driver Station]
 *
 * Be sure to select the correct feedback sensor using configSelectedFeedbackSensor() below.
 *
 * After deploying/debugging this to your RIO, first use the left Y-stick
 * to throttle the Talon manually.  This will confirm your hardware setup.
 * Be sure to confirm that when the Talon is driving forward (green) the
 * position sensor is moving in a positive direction.  If this is not the cause
 * flip the boolean input to the SetSensorPhase() call below.
 *
 * Once you've ensured your feedback device is in-phase with the motor,
 * use the button shortcuts to servo to target velocity.
 *
 * Tweak the PID gains accordingly.
 */
#include <iostream>
#include <string>

#include "frc/WPILib.h"
#include "ctre/Phoenix.h"
#include "Constants.h"

using namespace frc;

class Robot: public TimedRobot {
public:
	TalonFX * _talon = new TalonFX(3);
	Joystick * _joy = new Joystick(0);
	std::string _sb;
	int _loops = 0;

	void RobotInit() {
		_talon->ConfigFactoryDefault();
        /* first choose the sensor */
		_talon->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, kTimeoutMs);
		// _talon->SetSensorPhase(true);

		/* set the peak and nominal outputs */
		_talon->ConfigNominalOutputForward(0, kTimeoutMs);
		_talon->ConfigNominalOutputReverse(0, kTimeoutMs);
		_talon->ConfigPeakOutputForward(1, kTimeoutMs);
		_talon->ConfigPeakOutputReverse(-1, kTimeoutMs);
		/* set closed loop gains in slot0 */
		_talon->Config_kF(kPIDLoopIdx, 0.2, kTimeoutMs);
		_talon->Config_kP(kPIDLoopIdx, 0.05, kTimeoutMs);
		_talon->Config_kI(kPIDLoopIdx, 0.0, kTimeoutMs);
		_talon->Config_kD(kPIDLoopIdx, 0.0, kTimeoutMs);
	}
	/**
	 * This function is called periodically during operator control
	 */
	void TeleopPeriodic() {
		/* get gamepad axis */
		double leftYstick = _joy->GetY();
		double motorOutput = _talon->GetMotorOutputPercent();
		/* prepare line to print */
		_sb.append("\tout:");
		_sb.append(std::to_string(motorOutput));
		_sb.append("\tspd:");
		_sb.append(std::to_string(_talon->GetSelectedSensorVelocity(kPIDLoopIdx)));
		/* while button1 is held down, closed-loop on target velocity */
		if (_joy->GetRawButton(1)) {
        	/* Speed mode */
			/* Convert 500 RPM to units / 100ms.
			 * 4096 Units/Rev * 500 RPM / 600 100ms/min in either direction:
			 * velocity setpoint is in units/100ms
			 */
			double targetVelocity_UnitsPer100ms = leftYstick * 500.0 * 2048 / 600;
			/* 500 RPM in either direction */
        	_talon->Set(ControlMode::Velocity, targetVelocity_UnitsPer100ms); 

			/* append more signals to print when in speed mode. */
			_sb.append("\terrNative:");
			_sb.append(std::to_string(_talon->GetClosedLoopError(kPIDLoopIdx)));
			_sb.append("\ttrg:");
			_sb.append(std::to_string(targetVelocity_UnitsPer100ms));
        } else {
			/* Percent voltage mode */
			_talon->Set(ControlMode::PercentOutput, leftYstick);
		}
		/* print every ten loops, printing too much too fast is generally bad for performance */
		if (++_loops >= 10) {
			_loops = 0;
			printf("%s\n",_sb.c_str());
		}
		_sb.clear();
	}
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

```


### 閻㈠灚婧€閺勵垰鎸ф禒锝囩垳閵嗭拷

https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages