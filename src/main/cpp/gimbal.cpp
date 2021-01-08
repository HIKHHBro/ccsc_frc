#include "gimbal.h"


Gimbal::Gimbal(/* args */)
{
    //TODO: neo pwm使用
// m_leftMotor.Set(m_stick.GetRawAxis(0));
    std::cout<<"to be developed"<<std::endl;
}

Gimbal::~Gimbal()
{
    
}
///< 设置抓取伸缩
void Gimbal::set_snatch(bool sw)
{
    if(sw)
    {

    }
    else
    {
        /* code */
    }
    
}








#ifdef GIMBAL_DEBUG
void Gimbal::color_display(void)
{
    std::string colorString;
    switch (curr_color)
    {
    case B:
        colorString = "Blue";
        break;
    case G:
        colorString = "Green";
    case R:
        colorString = "Red";
        break;
    case Y:
        colorString = "Yellow";
        break;
    default:
        colorString = "Unknown";
        break;
    }
    frc::SmartDashboard::PutNumber("Red", detectedColor.red);
    frc::SmartDashboard::PutNumber("Green", detectedColor.green);
    frc::SmartDashboard::PutNumber("Blue", detectedColor.blue);
    frc::SmartDashboard::PutNumber("Confidence", confidence);
    frc::SmartDashboard::PutString("Detected Color", colorString);
}

#endif