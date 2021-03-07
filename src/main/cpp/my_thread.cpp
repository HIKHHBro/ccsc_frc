#include "my_thread.h"
#include <unistd.h>
#include <frc/smartdashboard/smartdashboard.h>
MyThread::MyThread()
{
 
}
 
MyThread::MyThread(int utime)
{
  loop_time = utime;
}
MyThread::~MyThread()
{
	if (!this->isInterrupted())
	{
		this->interrupt();
	}
  std::cout<<"~~~"<<std::endl;
}
 
void MyThread::start_detach()
{
  if (this->isInterrupted())
  {
    isInterript = false;
    std::thread thr(std::bind(&MyThread::run,this));
    this->th = std::move(thr);
    this->th.detach();
    thread_count++;
  }
}
void MyThread::start_join()
{
  if (this->isInterrupted())
  {
    isInterript = false;
    std::thread thr(std::bind(&MyThread::run,this));
    this->th = std::move(thr);
    this->th.join();
    thread_count++;
  }
}
std::thread::id MyThread::getId()
{
	return this->th.get_id();
}
 
void MyThread::interrupt()
{
	this->isInterript = true;
}
 
bool MyThread::isInterrupted()
{
	return this->isInterript;
}
void MyThread::run()
{
  std::cout << "MyThread" << std::endl;
  thread_sleep();
  interrupt();
}
void MyThread::set_loop_time(int t)
{
  loop_time = t;
}
void MyThread::thread_sleep()
{
    // timer_sleep(0,loop_time);
    usleep(loop_time);
}
///< 获取
float MyThread::get_loop_freq()
{
  return (1.0*1000.0 * 1000.0) /(float)loop_time;
}
void MyThread::thread_debug()
{
  wpi::StringRef status_str;
  frc::SmartDashboard::PutNumber("线程被创建次数",thread_count);
  if(isInterript == true)
    status_str = "shop";
  else status_str = "run";
  frc::SmartDashboard::PutString("线程运行转态",status_str);
}