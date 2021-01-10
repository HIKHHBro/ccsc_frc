#include "BaseModule.h"

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
void Motor::set_reduction_ratiop(int numb1 ,int numb2,int numb3)
{
    reduction_ratiop = float(numb3)/float(numb2)/float(numb1);
}
void Motor::set_reduction_ratiop(int numb1 ,int numb2,int numb3,int numb4)
{
    reduction_ratiop = float(numb4)/float(numb3)/float(numb2)/float(numb1);
}


Falcon::Falcon():Motor(6380,2048,1)
{

}
Falcon::~Falcon()
{

}
int Falcon::get_position_error(int target,int real)
{
    return (target - real);
}
//TODO: 待测试
float Falcon::enc_100ms_to_rpm(float enc)
{
    return enc / float(encoder_l) * 600.0; 
}
int Falcon::rpm_to_enc_100ms(float rpm)
{
    return int( rpm / 600.0 * encoder_l);
}
int Falcon::mm_to_enc(float len)
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

Neo::Neo(int channel,float k):SparkMax(channel),RampFunction(k)
{
    max_rpm = 5700;
    encoder_l = 4096;
}

Neo::~Neo()
{

}