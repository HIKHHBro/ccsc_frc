#include "status_led.h"

frc::PowerDistributionPanel Status_led::pdp{0};//电池
frc::Solenoid Status_led::solenoid[2]{{20,4},{20,5}};
int Status_led::low_battery_flag = 0;
std::queue<Status_led::status_led> Status_led::status_queue;
Status_led::Status_led(): MyThread(500000)
{
    start_detach();
}
Status_led::~Status_led()
{
    interrupt();
}
///< 设置状态指示模式
void Status_led::set_tip_mode(LAMP_MODE mode)
{
    if(!ls_mode(mode))
    {
        status_led led;
        switch (mode)
        {
        case LOW_Battery:
            led.mode = LOW_Battery;
            led.times = 20;
            led.led_status[0] = false;
            led.led_status[1] = true;
            status_queue.push(led);
            break;
        case NO_Reset:
            led.mode = NO_Reset;
            led.times = 10;
            led.led_status[0] = false;
            led.led_status[1] = false;
            status_queue.push(led);
            break;
        default:
            break;
        }
    }
}
///< 低电量提示
void Status_led::low_battery_tip()
{
    if(pdp.GetVoltage() <10)
    {
        low_battery_flag ++;
    }
    if(low_battery_flag > 4)
        set_tip_mode(LOW_Battery);
}

Status_led::LAMP_MODE test_mode;
void Status_led::test()
{
    LAMP_MODE get5 = (LAMP_MODE)frc::SmartDashboard::GetNumber("NO_Battery",test_mode);
    if(test_mode != get5)
        test_mode = get5;
}
void Status_led::test_dis()
{
    frc::SmartDashboard::PutNumber("NO_Battery",test_mode);
}
///< 线程函数的重写
void Status_led::run()
{
    while (!isInterrupted())
    {
        while (status_queue.empty() == true)
        {
            status_led status = status_queue.front();
            status_queue.pop();
            set_led_status(status);
        }
        thread_sleep();
    }
}
///< 设置灯状态
    void Status_led:: set_led_status(status_led &status)
{
    if(status.times > 20)
        status.times = 20;
    else if(status.times < 0)
        status.times = 0;
    status.delay = 1000000000 / status.times / 2;
    while (status.times > 0)
    {
        status.times--;
        status.led_status[0] = !status.led_status[0];
        status.led_status[1] = !status.led_status[1];
        solenoid[0].Set(status.led_status[0]);
        solenoid[1].Set(status.led_status[1]);
        usleep(status.delay);
    }
    sleep(1);
}
///< 匹配队列中是否有对应的模式
bool Status_led::ls_mode(LAMP_MODE mode)
{
    std::queue<status_led> q = status_queue;
    int size = q.size();
    for (int i=0; i<size; ++i)
    {
        status_led sl = q.front();
        if (sl.mode == mode)
        {
        return true;
        }
        q.pop();
    }
    return false;
}