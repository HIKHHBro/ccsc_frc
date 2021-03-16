#ifndef __SHOOT_H
#define __SHOOT_H
#include "BaseModule.h"
#include "ctre/Phoenix.h"
#include "my_thread.h"
#include <frc/DigitalInput.h>
#include <frc/smartdashboard/smartdashboard.h>
#include <frc/SerialPort.h>
class Shoot : public Falcon, public MyThread
{
public:
    enum MOTOR{Hor_tr,Hor_tr_u,Ver_tr,Sh1,Sh2,ALL};
    Shoot(int pwm_c,int can_id);
    ~Shoot();
    bool open_vertical_transfer();
    void open_horizontal_transfer();
    void close_vertical_transfer();
    void close_horizontal_transfer();
    void carry_out(MOTOR M,float rpm);
    void set_gimbal_angle(float angle);
    void set_gimbal_percent(float percent);
    bool reset();
    void start_shoot();
    void stop_shoot();
    bool stop_vertical_transfer();
    bool auto_shoot();
    float auto_cal_shoot_pitch_angle(float);
    TalonFX* gimbal_motor;
    bool get_reset_status(){return is_reseted;};
    int distance = 0;
    char buffer[10] = {0};
      ///< 计算校验和
    int cal_check_sum()
    {
        return ((buffer[0] + buffer[1] + buffer[2])&0x00FF);
    }
    ///< 更新超声波测距的距离
    void updata_distance()
    {
        memset(buffer,'\0',sizeof(buffer));
        ultrasonic->Read(buffer,4);
        frc::SmartDashboard::PutNumber("ultr buffer[0]",buffer[0]);
        frc::SmartDashboard::PutNumber("ultr buffer[1]",buffer[1]);
        frc::SmartDashboard::PutNumber("ultr buffer[2]",buffer[2]);
        frc::SmartDashboard::PutNumber("ultr buffer[3]",buffer[3]);
        if(buffer[0] == 0xFF && buffer[3] == cal_check_sum())
        {
            distance = (buffer[1]<<8) | (buffer[2]);
        }
        frc::SmartDashboard::PutNumber("distance d",distance);

    }
    ///< 测试超声波
    void test_ultrasonic()
    {

        frc::SmartDashboard::PutNumber("ultr buffer[0]",buffer[0]);
        frc::SmartDashboard::PutNumber("ultr buffer[1]",buffer[1]);
        frc::SmartDashboard::PutNumber("ultr buffer[2]",buffer[2]);
        frc::SmartDashboard::PutNumber("ultr buffer[3]",buffer[3]);
        frc::SmartDashboard::PutNumber("ultr distance",distance);
        memset(buffer,'\0',sizeof(buffer)); 
    }

    int tmp_angle = 0;
    float k_a = 35;
    float k_b = -0.0075;
    ///< 获取发射补偿角度
    float get_pitch_angle(){
        
        updata_distance();
        if(distance >1000 && distance < 5000)
        {
            tmp_angle = k_a * distance + k_b;
            tmp_angle = (tmp_angle) > (23) ? (23) : (tmp_angle);
            tmp_angle = (tmp_angle) < (0) ? (0) : (tmp_angle);
            
        }
        return tmp_angle;
    }
    void set_test_display();
    void set_test();
      float point[2][2] = {0};
#ifdef SHOOT_DEBUG
    void display() override;
    void debug()   override;
#endif
private:
    void run()   override;

    frc::DigitalInput* reset_sw;
    frc::DigitalInput* shoot_sw;
    frc::SerialPort *ultrasonic;
    Neo* motor[ALL];
    float acc[ALL] = {0.01,0.01,0.05,0.1,0.1};
    float smoothing = 0;
    float kp = 0.05;
    float kf = 0.06;
    float max_angle = 30;//云台最大转动角度
    float reset_speed = 100;//rpm 0~6000
    float reset_acc = 0;//rpm
    float reset_output = 0.1;//0~1
    float reset_current_thres = 10;//amps
    float reset_speed_thres = 30;//100ms转的刻度
    float reset_period = 10000;//us
    bool is_reseted = false;
    int reset_error_count = 0;
    int reset_error_thre =  (int)(1000000.0/reset_period * 0.3);
    int auto_shoot_wait_time = 0;
    int auto_shoot_wait_conster = 100;
    int shoot_count = 0;
    bool shoot_count_sw_flag = false;
/*
 * 电机方向:
 * Hor_tr: 顺时针 负
 * Ver_tr: 逆时针 正
 *    Sh1: 逆时针
 *    Sh2: 逆时针
 */
    float neo_speed[ALL] = {0.40,0.60,-0.55,-0.7,-0.7};//RPM 最大5700
    //  float neo_speed[ALL] = {0,0,0,0,0};//RPM 最大5700
};





#endif