#include "gimbal.h"


Gimbal::Gimbal(/* args */)
{
    i2cPort = frc::I2C::Port::kOnboard;
    m_colorSensor = new rev::ColorSensorV3(i2cPort);
    set_color_rgb();
}

Gimbal::~Gimbal()
{
    delete m_colorSensor;
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
///< 设置颜色对应的rgb值
void Gimbal::set_color_rgb(void)
{
    color_target[B] = frc::Color(0.143, 0.427, 0.429);
    color_target[G] = frc::Color(0.197, 0.561, 0.240);
    color_target[R] = frc::Color(0.561, 0.232, 0.114);
    color_target[Y] = frc::Color(0.361, 0.524, 0.113);
    for(int i = 0;i<ALL_COLOR;i++)
        m_colorMatcher.AddColorMatch(color_target[i]);
}
Gimbal::COLOR Gimbal::get_target_color(void)
{
    detectedColor = m_colorSensor->GetColor();
    frc::Color matchedColor = m_colorMatcher.MatchClosestColor(detectedColor, confidence);
    for(int i = 0;i<ALL_COLOR;i++)
    {
        if(matchedColor == color_target[i])
        {
            curr_color = COLOR(i);
            return curr_color;
        }
    }
    curr_color = ALL_COLOR;
    return ALL_COLOR;
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