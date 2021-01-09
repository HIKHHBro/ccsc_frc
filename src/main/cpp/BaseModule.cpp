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
RampFunction::RampFunction(float k){this->k = k;}

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


Falcon::Falcon()
{
    encoder_l = 1023;
    max_rpm = enc_to_rpm(encoder_l);
    // reduction_ratiop = redu;
}
Falcon::~Falcon()
{

}\
//TODO: 待测试
float Falcon::enc_to_rpm(float enc)
{
    return enc / float(encoder_l) * 600.0; 
}
int Falcon::rpm_to_enc(float rpm)
{
    return int( rpm / 600.0 * encoder_l);
}
Neo::Neo()
{
    max_rpm = 5700;
    encoder_l = 4096;
}
Neo::~Neo()
{

}