# ifndef _GIMBAL_H
#define _GIMBAL_H
#include "BaseModule.h"
#include <frc/util/color.h>
#include "rev/ColorSensorV3.h"
#include "rev/ColorMatch.h"
#ifdef GIMBAL_DEBUG
#include <frc/smartdashboard/smartdashboard.h>
#endif
class Gimbal
{
private:
    frc::I2C::Port i2cPort;
    rev::ColorSensorV3 *m_colorSensor;
    rev::ColorMatch m_colorMatcher;
    frc::Color color_target[4];
    frc::Color detectedColor;
    double confidence = 0.0;
public:
    Gimbal(/* args */);
    ~Gimbal();

    /* 属性 */
    enum COLOR{B,G,R,Y,ALL_COLOR};
    COLOR curr_color;//当前颜色
    /* 方法 */
    void set_snatch(bool);
    void set_color_rgb(void);
    COLOR get_target_color(void);
    



#ifdef GIMBAL_DEBUG
    void color_display(void);
#endif
    

};




#endif