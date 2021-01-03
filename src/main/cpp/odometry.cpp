#include "odometry.h"

Odometry::Odometry() 
{
    rate_encoder = 2*3.1415926*r / sum_encoder;
}

void Odometry::cal_distance()
{
    Theta = _cal_angle();
    x += delta_d * cos(Theta);
    y +=  delta_d * sin(Theta);
}

float Odometry::_cal_angle()
{
    return 0.0;
}



