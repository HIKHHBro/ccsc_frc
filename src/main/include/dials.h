#ifndef __DIALS_H
#define __DIALS_H
#include "BaseModule.h"
#include <frc/util/color.h>
#include "rev/ColorSensorV3.h"
#include "rev/ColorMatch.h"
#include "ctre/Phoenix.h"
#include <atomic>
#ifdef DIALS_DEBUG
#include <frc/smartdashboard/smartdashboard.h>
#endif


class Dials
{
private:
    frc::I2C::Port i2cPort;
    rev::ColorSensorV3 *m_colorSensor;
    rev::ColorMatch m_colorMatcher;
    frc::Color color_target[4];
    frc::Color detectedColor;
    double confidence = 0.0;
    TalonFX* motor;
    float dials_d = 810;          //转盘直径 mm
    float color_angle = 45;      //每个颜色对应的角度 单位度
    float curr_position;
    float dials_perimeter = dials_d * 3.1415; //圆周长mm
    float frictiongear_d = 76.2; //摩擦轮直径 mm
    float arc_length = dials_perimeter / (360.0/color_angle); //每个颜色对应的弧长 单位mm
    float spin_control_vel;
    int serson = 2048;
    double reduction = reduction_ratio(frictiongear_d,dials_d);
    std::atomic<bool> spin_control_thread = false;
    std::atomic<bool> is_finished_spin_control = false;
    int color_tran_count = 0;
    /* 显示调试的变量 */
    int c_numb_serson_return;
    int spin_pos_error;
    int is_finished_spin_pos_err = (float)(c_numb_serson((1/float(ALL_COLOR))))/4.0;
public:
  Dials();
  Dials(int deviceNumber);
  ~Dials();


    /* 属性 */
    enum COLOR{B,G,R,Y,ALL_COLOR};
    COLOR curr_color;//当前颜色
    int can_id;
    float spin_numb_comp = 0;
    COLOR color_sequence_pre;
    /* 方法 */
    
    void set_color_rgb(void);
    COLOR get_color(void);
    
    bool dials_init();
    bool rotate_dials(COLOR);
    void fx_motor_magic(float kf,float kp,float kd);
    void spin_control(float numb);
    int c_numb_serson(float);
    bool spin_control_is_is_finished(void);
#ifdef DIALS_DEBUG
    void display(void);
    void set_para();
    bool color_sequence_check(COLOR curr);

#endif

};


#endif;