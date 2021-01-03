
#pragma once

#include <frc/TimedRobot.h>

#include "RobotContainer.h"
class Odometry : public frc::TimedRobot {
 public:
    Odometry();
    void cal_distance();

    float x;
    float y;
    float angle;

    
 private:
    float inc_encoder;
    float rate_encoder;
    float sum_encoder;
    float encoder_now;
    float encoder_last;
    float delta_d;
    float r;
    float Theta;

    float _cal_angle();
  // Have it null by default so that if testing teleop it
  // doesn't have undefined behavior and potentially crash.
};
