#include "my_thread.h"
#include <unistd.h>
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
  }
  else  std::cout << "is created" << std::endl;
}
void MyThread::start_join()
{
  if (this->isInterrupted())
  {
    isInterript = false;
    std::thread thr(std::bind(&MyThread::run,this));
    this->th = std::move(thr);
    this->th.detach();
  }
  else  std::cout << "is created" << std::endl;
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
}
void MyThread::set_loop_time(int t)
{
  loop_time = t;
}
void MyThread::thread_sleep()
{
    usleep(loop_time);
}