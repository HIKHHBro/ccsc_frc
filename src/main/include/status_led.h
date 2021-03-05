#ifndef __STATUS_LED_H
#define __STATUS_LED_H
#include <frc/Solenoid.h>
#include <queue>
#include <frc/PowerDistributionPanel.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "BaseModule.h"
#include "my_thread.h"
class Status_led :public MyThread
{

public:
    Status_led();
    ~Status_led();
    enum LAMP_MODE {LOW_Battery,NO_Reset,ALL_MODE};
    ///< 设置状态指示模式
    static void set_tip_mode(LAMP_MODE mode);
    ///< 低电量提示
    static void low_battery_tip();
    static void test();
    static void test_dis();
    struct status_led
    {
        int times = 0;
        bool led_status[2] = {false};
        LAMP_MODE mode = {ALL_MODE}; 
        int delay = 0;
    };//正在执行状态的参数
private:
    
    ///< 线程函数的重写
    void run() override;
    ///< 设置灯状态
    static void set_led_status(status_led &status);
    ///< 匹配队列中是否有对应的模式
    static bool ls_mode(LAMP_MODE mode);
    static int low_battery_flag;
    static std::queue<status_led> status_queue;// 状态队列
    static frc::PowerDistributionPanel pdp;//电池
    static frc::Solenoid solenoid[2];
};

#endif