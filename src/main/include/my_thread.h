#ifndef __MY_THREAD_H
#define __MY_THREAD_H
#include "BaseModule.h"
#include <atomic>
#include <thread>
#include "ctre/Phoenix.h"
class MyThread
{

public:
	MyThread();
  MyThread(int utime);
	virtual ~MyThread();
 
	void start();
	std::thread::id getId();
	void interrupt();
	bool isInterrupted();
  virtual void run();
  int loop_time = 30000;//微秒
private:
	std::atomic<bool> isInterript = false;
  std::thread th;
  

  
};

#endif
