#include "status_led.h"


Status_led::Status_led(): MyThread(50000)
{
    solenoid[0] = new frc::Solenoid(20,4);
    solenoid[1] = new frc::Solenoid(20,5);
    start_detach();
}
Status_led::~Status_led()
{
    interrupt();
}
///< 设置状态指示模式
bool fafd = false;
void Status_led::set_tip_mode(LAMP_MODE mode)
{
    if(!ls_mode(mode))
    {
        status_led led;
        switch (mode)
        {
        case LOW_Battery:
            led.mode = LOW_Battery;
            led.times = 5;
            led.led_status[0] = false;
            led.led_status[1] = true;
            status_queue.push(led);
            break;
        case NO_Reset:
            led.mode = NO_Reset;
            led.times = 2;
            led.led_status[0] = false;
            led.led_status[1] = false;
            status_queue.push(led);
            break;
        default:
            break;
        }
        fafd = false;
    }
    else
    {
        fafd = true;
        
    }
    frc::SmartDashboard::PutNumber("status",fafd);
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

Status_led::LAMP_MODE test_mode = Status_led::LOW_Battery;
void Status_led::test()
{
    LAMP_MODE get5 = (LAMP_MODE)frc::SmartDashboard::GetNumber("NO_Battery",test_mode);
        if(test_mode != get5)
        test_mode = get5;
    frc::SmartDashboard::PutNumber("q len",status_queue.size());
    thread_debug();

}
void Status_led::test_dis()
{
    frc::SmartDashboard::PutNumber("NO_Battery",test_mode);
    set_tip_mode(test_mode);
    set_tip_mode(NO_Reset);
}
///< 线程函数的重写
void Status_led::run()
{
    while (!isInterrupted())
    {
        while (!status_queue.empty())
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
    frc::SmartDashboard::PutNumber("status.times",status.times);
    if(status.times < 0)
        status.times = 0;
    else
    {
        if(status.times > 20)
        status.times = 20;
        status.delay = 1000000 / status.times / 2;
        frc::SmartDashboard::PutNumber("status.delay",status.delay);
        while (status.times > -1)
        {
            status.times--;
            status.led_status[0] = !status.led_status[0];
            status.led_status[1] = !status.led_status[1];
            solenoid[0]->Set(status.led_status[0]);
            solenoid[1]->Set(status.led_status[1]);
            std::cout<<status.times<<std::endl;
            timer_sleep(0,status.delay);
            // timer_sleep(0,status.delay);
            frc::SmartDashboard::PutNumber("status.times",status.times);
        }
        std::cout<<status.times<<std::endl;
        solenoid[0]->Set(false);
        solenoid[1]->Set(false);
        timer_sleep(1,0);
        // sleep(1);
    }

     

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