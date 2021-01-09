#include "lifting.h"
Lifting::Lifting(/* args */)
{
    for(int i =0;i<M_ALL;i++)
    {
        motor[i] = new TalonFX(1);  

        /* Factory default hardware to prevent unexpected behavior */
        motor[i]->ConfigFactoryDefault();

        /* Configure Sensor Source for Pirmary PID */
        motor[i]->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor,0, 10);

        motor[i]->SetInverted(TalonFXInvertType::CounterClockwise);

        /* Set relevant frame periods to be at least as fast as periodic rate */
        motor[i]->SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
        motor[i]->SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);

        /* Set the peak and nominal outputs */
        motor[i]->ConfigNominalOutputForward(0, 10);
        motor[i]->ConfigNominalOutputReverse(0, 10);
        motor[i]->ConfigPeakOutputForward(1, 10);
        motor[i]->ConfigPeakOutputReverse(-1, 10);

        /* Set Motion Magic gains in slot0 - see documentation */
        motor[i]->SelectProfileSlot(0, 0);
        motor[i]->Config_kF(0, 0.3, 10);
        motor[i]->Config_kP(0, 0.1, 10);
        motor[i]->Config_kI(0, 0.0, 10);
        motor[i]->Config_kD(0, 0.0, 10);

        /* Set acceleration and vcruise velocity - see documentation */
        motor[i]->ConfigMotionCruiseVelocity(1500, 10);
        motor[i]->ConfigMotionAcceleration(1500, 10);

        /* Zero the sensor */
        motor[i]->SetSelectedSensorPosition(0, 0, 10);
        motor[i]->ConfigMotionSCurveStrength(smoothing, 0);
    }
    motor[0]->Follow(*motor[1],FollowerType::FollowerType_AuxOutput1);
}           


Lifting::~Lifting()
{
}
//TODO: 单位转换
///< 设置电机执行
void Lifting::set_point(float len)
{
    motor[0]->Set(ControlMode::MotionMagic, len);
    motor[1]->Set(ControlMode::MotionMagic, len);
}
///< 单位转换



#ifdef LIFT_DEBUG
void Lifting::display()
{

}
void Lifting::debug()
{

}
#endif