#ifndef __DIALS_H
#define __DIALS_H
#include "BaseModule.h"
#include <frc/util/color.h>
#include "rev/ColorSensorV3.h"
#include "rev/ColorMatch.h"
#include "ctre/Phoenix.h"
#include <atomic>
#include "my_thread.h"
#include <frc/Solenoid.h>
#include <frc/DigitalInput.h>
#ifdef DIALS_DEBUG
#include <frc/smartdashboard/smartdashboard.h>
#endif

class Dials:public MyThread,public Falcon
{

public:
  Dials();
  Dials(int deviceNumber);
  ~Dials();


    /* 属性 */
    enum COLOR{B,G,R,Y,ALL_COLOR};
    enum MODE{Spin,Pos};
    MODE state = Spin;
    COLOR curr_color;//当前颜色
    int can_id;
    float spin_numb_comp = 0;
    COLOR color_sequence_pre = B;
    
    /* 方法 */
    
    void set_color_rgb(void);
    COLOR get_color(void);
    
    bool dials_init();
    bool rotate_dials(COLOR);
    void fx_motor_magic(float kf,float kp,float kd);
    int c_numb_serson(float);
    bool spin_control_is_finished(void);
    bool color_sequence_check(COLOR curr);
    bool start_spin_control(float numb);
    void start_pos_control(COLOR);
    float optimal_path(COLOR target,COLOR curr);
    bool color_is_changed(COLOR curr);
    void lift();
    void put_down();
    bool is_arrived();
#ifdef DIALS_DEBUG
    void display(void);
    void set_para();
#endif
private:
    frc::I2C::Port i2cPort;
    rev::ColorSensorV3 *m_colorSensor;
    rev::ColorMatch m_colorMatcher;
    frc::Color color_target[4];
    frc::Color detectedColor;
    frc::Solenoid *solenoid[2];
    frc::DigitalInput* is_arrived_sw;
    double confidence = 0.0;
    TalonFX* motor;
    float dials_d = 810;          //转盘直径 mm
    float color_angle = 45;      //每个颜色对应的角度 单位度
    float curr_position;
    float dials_perimeter = dials_d * 3.1415; //圆周长mm
    float frictiongear_d = 100; //摩擦轮直径 mm
    float arc_length = dials_perimeter / (360.0/color_angle); //每个颜色对应的弧长 单位mm
    float spin_control_vel = 1000;
    double reduction = reduction_ratio(frictiongear_d,dials_d);
    int color_tran_count = 0;
    /* 显示调试的变量 */
    int c_numb_serson_return = 0;
    int spin_pos_error = 0;
    float is_finished_spin_pos_err = (float)(angle_to_enc(45.0))/1.5;
    float spin_numb = 0;
    COLOR target_color; //需要旋转到的指定颜色
    bool is_finished_spin_control = false;
    bool is_finished_pos_control = false;
    int reset_period = 5000;//5ms
    int time_count[2] = {0,0};
    float wait_time[2] = {2,3};
    int time_thre[2];
    int direction = -1;//和电机方向相反
    float target_angle;
    COLOR color_temp_last;

    /* 私有方法 */
    void spin_control_thread();
    void pos_control_thread();
    void run() override;
};
#endif