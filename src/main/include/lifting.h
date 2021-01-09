#ifndef __LIFTING_H
#define __LIFTING_H
#include "BaseModule.h"
#include "ctre/Phoenix.h"
#ifdef LIFT_DEBUG
#include <frc/smartdashboard/smartdashboard.h>
#endif

class Lifting : public Base
{
private:
    enum MOTOR {M1,M2,M_ALL};
    TalonFX* motor[2];
    float smoothing = 0;
public:
    Lifting(/* args */);
    ~Lifting();
    void set_point(float len);
    

#ifdef LIFT_DEBUG
    void display() override;
    void debug() override;
#endif
};


#endif