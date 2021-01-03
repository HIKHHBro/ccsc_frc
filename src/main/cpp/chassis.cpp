/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include "chassis.h"
#include "Math.h"
Chassis::Chassis()
{
    try
    {
        ahrs = new AHRS(SPI::Port::kMXP);
        ramp_func = new RampFunction(5);
        for(int i=0;i<M_ALL;i++)
        {
            motor[i] = new TalonFX(i+1);  
            motor[i]->ConfigFactoryDefault();
            /* first choose the sensor */
            motor[i]->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 0);
            // motor[i]->SetSensorPhase(true);
            motor[i]->ConfigNeutralDeadband(0);
            /* set the peak and nominal outputs */
            motor[i]->ConfigNominalOutputForward(0, 0);
            motor[i]->ConfigNominalOutputReverse(0, 0);
            motor[i]->ConfigPeakOutputForward(1, 0);
            motor[i]->ConfigPeakOutputReverse(-1, 0);
            /* set closed loop gains in slot0 */
            motor[i]->Config_kF(0, 0.6, 0);
            motor[i]->Config_kP(0, 0.25, 0);
            motor[i]->Config_kI(0, 0.0, 0);
            motor[i]->Config_kD(0, 0.7, 0);
            motor[i]->ConfigClosedLoopPeriod(0,1,0);
            motor[i]->ConfigVelocityMeasurementPeriod(VelocityMeasPeriod::Period_100Ms,0);
            // fx->ConfigClosedloopRamp(0.5,0);
        }
        
        auto_run_map_pid[x] = new frc2::PIDController(5.0,0.02,2.0);
        auto_run_map_pid[y] = new frc2::PIDController(5.0,0.02,2.0);
        auto_run_map_pid[z] = new frc2::PIDController(5.0,0.02,2.0);


    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    
}
Chassis::~Chassis()
{
    delete ahrs;
}
#if 0
//TODO: 待测试  1、机体坐标系 2、世界坐标系
///< 底盘运动模型正解
void Chassis::motion_model(int csys,float vx,float vy,float vz)
{
    float vx_m,vy_m,w_m = 0;
    float a = (45.0)/180.0*3.1415926; 
    if(check_gyro())
    {
        w_m = (ahrs->GetYaw())/180.0*3.1415926; 
        vx_m = cos(w_m) *vx + sin(w_m) * vy;
        vy_m = -sin(w_m) * vx + cos(w_m) * vy;  
    }
   else
   {
        vx_m = vx;
        vy_m = vy;
   }
    speed[MOTOR1] = +cos(a)*vy_m + sin(a)*vx_m - R*vz;//1******2	 
	speed[MOTOR2] = -cos(a)*vy_m + sin(a)*vx_m - R*vz;//********
	speed[MOTOR3] = -cos(a)*vy_m - sin(a)*vx_m - R*vz;//********
	speed[MOTOR4] = +cos(a)*vy_m - sin(a)*vx_m - R*vz;//4******3	 
    std::cout<< "MOTOR1:"<<speed[M1]<<"\t"<<"M2:"<<speed[M2]<<"\t"<<"M3:"<<speed[M3]<<"\t"<<"M4:"<<speed[M4]<<cos(0.25*3.1415)<<"\n";
}
#endif
#if 1
//TODO: 待测试  1、机体坐标系 2、世界坐标系
///< 底盘运动模型正解
void Chassis::motion_model(float vx,float vy,float vz)
{
    float w = 0;
    float x = wheel_rc_to_sensor(vx);
    float y = wheel_rc_to_sensor(vy);
    float z = w_rc_to_sensor(vz);
 
	// speed[M1] = -sin(DEG_TO_RAD(wheel_theta - w)) * vx  - cos(DEG_TO_RAD(wheel_theta - w)) * vy + chassis_r  * vz;//  ********
    // speed[M4] = -sin(DEG_TO_RAD(wheel_theta + w)) * vx  + cos(DEG_TO_RAD(wheel_theta + w)) * vy + chassis_r  * vz;//2******1
	// speed[M2] =  sin(DEG_TO_RAD(wheel_theta + w)) * vx  - cos(DEG_TO_RAD(wheel_theta + w)) * vy + chassis_r  * vz;//********
	// speed[M3] =  sin(DEG_TO_RAD(wheel_theta - w)) * vx  + cos(DEG_TO_RAD(wheel_theta - w)) * vy + chassis_r  * vz;//3******4
    if(reference == WORLD)
    {
        if(check_gyro())
        {
            w = vz;
        }
        else 
        {
            std::cout<<"陀螺仪掉线"<<std::endl;
            reference = CAR;
        }
    }
    else w = 0;

    speed[M1] = -sin(DEG_TO_RAD(wheel_theta - w))* x + cos(DEG_TO_RAD(wheel_theta - w))* y + chassis_r * z;//4******1
    speed[M2] = -sin(DEG_TO_RAD(wheel_theta + w))* x - cos(DEG_TO_RAD(wheel_theta + w))* y + chassis_r * z;//********
    speed[M3] =  sin(DEG_TO_RAD(wheel_theta - w))* x - cos(DEG_TO_RAD(wheel_theta - w))* y + chassis_r * z;//********
    speed[M4] =  sin(DEG_TO_RAD(wheel_theta + w))* x + cos(DEG_TO_RAD(wheel_theta + w))* y + chassis_r * z;//3******2


    std::cout<< "MOTOR1:"<<speed[M1]<<"\t"<<"M2:"<<speed[M2]<<"\t"<<"M3:"<<speed[M3]<<"\t"<<"M4:"<<speed[M4]<<cos(0.25*3.1415)<<"\n";
}
#endif
bool Chassis::check_gyro()
{
    return ahrs->IsConnected();
}
///< 获取运动目标值
void Chassis::get_run_target(float* target)
{

}
///< 更新遥控指令
void Chassis::update_rc_data()
{

}

void Chassis::rc_run(float vx,float vy,float vz)
{

    motion_model(vx,vy,vz);
    // std::cout<< "s1"<<speed[0] <<"s2"<<speed[1] <<"s3"<< speed[2] << "s4"<<speed[3]<<std::endl;
    for(int i=0;i<M_ALL;i++)
        motor[i]->Set(ControlMode::Velocity,ramp_func->set(speed[i]));
}

///< 位置伺服


///<< 里程计计算
bool Chassis::milemter()
{
    if(check_gyro())
    {
        float w = ahrs->GetYaw();
        milemeter[x]  = -sin(DEG_TO_RAD(wheel_theta - w)) *  wheel_s[M1] - sin(DEG_TO_RAD(wheel_theta + w)) * wheel_s[M2] + sin(DEG_TO_RAD(wheel_theta - w)) * wheel_s[M3] + sin(DEG_TO_RAD(wheel_theta + w)) * wheel_s[M4];
        milemeter[y]  =  cos(DEG_TO_RAD(wheel_theta - w)) * wheel_s[M1] - cos(DEG_TO_RAD(wheel_theta + w)) * wheel_s[M2] - cos(DEG_TO_RAD(wheel_theta - w)) * wheel_s[M3] + cos(DEG_TO_RAD(wheel_theta = w))* wheel_s[M4];
        milemeter[z] =  wheel_s[M1] +   wheel_s[M2] +   wheel_s[M3] +   wheel_s[M4];
        return true;
    }
    else return false;

}
///<< 线数转换成mm
float  Chassis::series_to_mm(int16_t wheel)
{
    return (float)wheel*(2*wheel_r * 3.1415926 / motor_series);
}
///<位置控制
bool Chassis::to_position(float x,float y,float w)
{
    return true;
}

///< 控制轮子单位转换  遥控值转为传感器线数 s/100ms
int Chassis::wheel_rc_to_sensor(float value)
{
    if (abs(value)<=1)
    {
        return int(value * motor_series* rc_control_x_y);
    }
    else
    {
        std::cout<<"遥控数据值过大"<<std::endl;
        return 0;
    }  
}
///< 设置坐标系 
void Chassis::set_reference(Reference ref)
{
    reference = ref;
}
//TODO: 设置量程范围
///< 旋转角度获取目标值量程转换
float Chassis::w_rc_to_sensor(float value)
{
    if (abs(value)<=1)
    {
        return int(value * motor_series* rc_control_w);
    }
    else
    {
        std::cout<<"遥控数据值过大"<<std::endl;
        return 0;
    }  
}
//TODO: 哪个才是当前编码器的值
///<更新和累计编码器值
void Chassis::updata_series(void)
{
    for (int i = 0;i<M_ALL;i++)
    {
        wheel_s[i] = motor[i]->GetSelectedSensorPosition() - series_position[i];
    }
}
//TODO: 待完善
///<重设编码器值
void Chassis::set_series(int value)
{
    for(int i = 0;i<M_ALL;i++)
        series_position[i] = motor[i]->GetSelectedSensorPosition() + value;
}
///< 待测试
///<自动阶段
bool Chassis::auto_run(void)
{
    float output[3];
    for(int i =1;i<map_len;i++)
    {
        if(abs(auto_run_map_pid[y]->GetPositionError()) < abs(map[i][y] - map[i][y-1])/2)
        {
            output[x] = auto_run_map_pid[x]->Calculate(milemeter[x],map[i][x]);
            output[y] = auto_run_map_pid[y]->Calculate(milemeter[y],map[i][y]);
            output[z] = auto_run_map_pid[z]->Calculate(milemeter[z],map[i][z]);
        }
        else i--;
    }
    rc_run(output[x],output[y],output[z]);
    
}