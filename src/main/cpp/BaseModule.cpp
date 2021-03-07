#include "BaseModule.h"
#include <frc/smartdashboard/smartdashboard.h>

// Base::Base(){}

// Base::~Base(){}
void Base::display()
{
    std::cout<<"display"<<std::endl;
}
void Base::debug()
{
    std::cout<<"debug"<<std::endl;
}
float Base::get_number(wpi::StringRef keyName, double value,double min,double max)
{
    float value_temp = value;
    float get = frc::SmartDashboard::GetNumber(keyName,value_temp);
    if(get != value_temp)
    {
        value_temp = limit(get,min,max);
    }
    frc::SmartDashboard::PutNumber(keyName,value_temp);
    return value_temp;
}

int Base::get_number(wpi::StringRef keyName, int value,int min,int max)
{
    int value_temp = value;
    int get = frc::SmartDashboard::GetNumber(keyName,value_temp);
    if(get != value_temp)
    {
        value_temp = limit(get,min,max);
    }
    frc::SmartDashboard::PutNumber(keyName,value_temp);
    return value_temp;
}


RampFunction::RampFunction(float k):k(k){last_value = 0;}

RampFunction::~RampFunction(){}

Motor::Motor()
{

}
Motor::~Motor()
{

}
Motor::Motor(double rpm,int enc,float redu)
{
    max_rpm = rpm;
    encoder_l = enc;
    reduction_ratiop = redu;
}
///< 百分比转转每分钟
double Motor::per_to_rpm(float per)
{
    return per * max_rpm;
}
///< 转每分钟转百分比 
float Motor::rpm_to_per(float rpm)
{
    return rpm / max_rpm;
}
///< //numb1 为输入, numbx 为最后电机真实输出
void Motor::set_reduction_ratiop(float numb)
{
    reduction_ratiop = numb;
}
void Motor::set_reduction_ratiop(float numb1,float numb2)
{
    reduction_ratiop = (numb2/numb1);
}
void Motor::set_reduction_ratiop(int numb1,int numb2) 
{
    reduction_ratiop = float(numb2)/float(numb1);
}
void Motor::set_reduction_ratiop(int numb1 ,int numb2,int numb3,int numb4)
{
    reduction_ratiop = (float(numb4)/float(numb3)) * (float(numb2)/float(numb1));
}


Falcon::Falcon():Motor(6000,2048,1)
{

}
Falcon::~Falcon()
{

}
int Falcon::get_position_error(int target,int real)
{
    return (target - real);
}
float Falcon::enc_100ms_to_rpm(float enc)
{
    return enc / float(encoder_l) * 600.0; 
}
float Falcon::rpm_to_enc_100ms(float rpm)
{
    return ( rpm / 600.0 * encoder_l);
}
float Falcon::mm_to_enc(float len)
{
    return len *  (((float)encoder_l * reduction_ratiop) / (d * 3.1416));
}
double Falcon::enc_to_mm(int enc)
{
    return (float)enc /  (((float)encoder_l * reduction_ratiop) / (d * 3.1416));
}
void Falcon::set_dia(float len)
{
    d = len;
}
///< mm/s 转 enc/100ms
float Falcon::mms_to_enc100ms(float mms)
{
    return ( (mms * reduction_ratiop * 0.1 * (float)encoder_l ) / (3.14 * d));
}
///<  enc/100ms 转 mm/s
float Falcon::enc100ms_to_mms(int enc)
{
    float enc_temp = (float)enc;
    return ((enc_temp / 0.1) / (float)encoder_l) * d * 3.145 / reduction_ratiop;
}



Neo::Neo(int channel,float k):SparkMax(channel),RampFunction(k)
{
    max_rpm = 5700;
    encoder_l = 4096;
}

Neo::~Neo()
{

}

void timer_sleep(unsigned long seconds,unsigned long mseconds)
{
    struct timeval tv;
    tv.tv_sec=seconds;
    tv.tv_usec=mseconds;
    int err;
    do{
       err=select(0,NULL,NULL,NULL,&tv);
    }while(err<0 && errno==EINTR);
}