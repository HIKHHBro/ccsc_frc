#ifndef __MY_THREAD_H
#define __MY_THREAD_H
#include "BaseModule.h"
#include <atomic>
#include <thread>
#include "ctre/Phoenix.h"
#include <unistd.h>
class MyThread
{
public:
	MyThread();
	MyThread(int utime);
	virtual ~MyThread();
	void start_detach();
	void start_join();
	std::thread::id getId();
	void interrupt();
	bool isInterrupted();
	virtual void run();
	void set_loop_time(int t);
	float get_loop_freq();
	void thread_sleep();
private:
	std::atomic<bool> isInterript = true;
	std::thread th;
	int loop_time = 30000;//微秒
  

  
};

#endif
