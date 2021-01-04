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
        ahrs = new AHRS(SPI::Port::kMXP);//陀螺仪
        ramp_func = new RampFunction(5);//斜坡函数
        motor_init();//电机初始化
        set_reference(CAR);//坐标系设置
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
    float x = wheel_rc_to_sensor(vx); //-819~819 线数
    float y = wheel_rc_to_sensor(vy);//-819~819 线数
    float z = w_rc_to_sensor(vz); //-204~204 线数
    /*  */ 
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
    speed[M1] =  sin(DEG_TO_RAD(wheel_theta + w))* x \
                -cos(DEG_TO_RAD(wheel_theta + w))* y + chassis_r * z;//2******1
    speed[M2] =  sin(DEG_TO_RAD(wheel_theta - w))* x \
                +cos(DEG_TO_RAD(wheel_theta - w))* y + chassis_r * z;//********
    speed[M3] = -sin(DEG_TO_RAD(wheel_theta + w))* x \
                +cos(DEG_TO_RAD(wheel_theta + w))* y + chassis_r * z;//********
    speed[M4] = -sin(DEG_TO_RAD(wheel_theta - w))* x \
                -cos(DEG_TO_RAD(wheel_theta - w))* y + chassis_r * z;//3******4
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

    motion_model(vx,vy,vz);//-1~1
    for(int i=0;i<M_ALL;i++)
        motor[i]->Set(ControlMode::Velocity,ramp_func->set(speed[i]));
}


//TODO:待测试
///<< 里程计计算
bool Chassis::milemter()
{
    if(check_gyro())
    {
        float w = ahrs->GetYaw();
        milemeter[x]  =   sin(DEG_TO_RAD(wheel_theta + w)) * series_to_mm(wheel_s[M1]) \
                        + sin(DEG_TO_RAD(wheel_theta - w)) * series_to_mm(wheel_s[M2]) \
                        - sin(DEG_TO_RAD(wheel_theta + w)) * series_to_mm(wheel_s[M3]) \
                        - sin(DEG_TO_RAD(wheel_theta - w)) * series_to_mm(wheel_s[M4]);
        milemeter[y]  = - cos(DEG_TO_RAD(wheel_theta + w)) * series_to_mm(wheel_s[M1]) \
                        + cos(DEG_TO_RAD(wheel_theta - w)) * series_to_mm(wheel_s[M2]) \
                        + cos(DEG_TO_RAD(wheel_theta + w)) * series_to_mm(wheel_s[M3]) \
                        - cos(DEG_TO_RAD(wheel_theta - w)) * series_to_mm(wheel_s[M4]);
        milemeter[z] =  series_to_mm(wheel_s[M1]) +   series_to_mm(wheel_s[M2]) + \
                        series_to_mm(wheel_s[M3]) +   series_to_mm(wheel_s[M4]);
        std::cout<<" milemeter[x]= "<< milemeter[x]\
                 <<" milemeter[y]= "<< milemeter[y]\
                 <<" milemeter[z]= "<< milemeter[z]<<std::endl;
        return true;
    }
    else 
    {
        std::cout<<"无陀螺仪"<<std::endl;
        return false;
    }
   

}
//TODO:待测试
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
//TODO: 待测试pid  新加了pid初始化
//DONE: 完成线程测试
///<自动阶段
void Chassis:: auto_run()
{
    float output[3];
    auto_run_map_pid[x] = new frc2::PIDController(5.0,0.02,5.0);
    auto_run_map_pid[y] = new frc2::PIDController(5.0,0.02,5.0);
    auto_run_map_pid[z] = new frc2::PIDController(5.0,0.02,5.0);
    for(int i =1;(i<map_len)||!auto_run_is_finished;i++)
    {
        milemter();
        try
        {
            if(abs(auto_run_map_pid[y]->GetPositionError()) > abs(map[i][y-1] - map[i][y])/2)
            {
                i--;
            }
            output[x] = auto_run_map_pid[x]->Calculate(milemeter[x],map[i][x]);
            output[y] = auto_run_map_pid[y]->Calculate(milemeter[y],map[i][y]);
            output[z] = auto_run_map_pid[z]->Calculate(milemeter[z],map[i][z]);
            std::cout<<"output[x]="<<output[x]\
                     <<"output[y]="<<output[y]\
                     <<"output[z]="<<output[z]<<std::endl;
            // rc_run(output[x],output[y],output[z]);
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
        usleep(30000);
        std::cout<< "this auto run"<<"i="<<i<<std::endl;
    }
        auto_run_status = false;
        auto_run_is_finished = true;
        std::cout<< "exit"<<std::endl;

}

//DONE: 完成测试
///< 开启自动模式
bool Chassis::start_auto_run(void)
{
    if(!auto_run_status)
    {
        std::thread auto_run_pid(&Chassis::auto_run,this);
        auto_run_pid.detach();
        auto_run_status = true;
        auto_run_is_finished = false;
        return true;
    }

}

///< 获取自动阶段状态
bool Chassis::get_auto_run_status()
{
    return auto_run_status;
}
///< 退出自动模式
bool Chassis::exit_auto_run()
{
    auto_run_is_finished = true;
}

///< 电机初始化
void Chassis::motor_init()
{
    
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
}
//TODO: 待测试
///< 获取自动模式是否完成
bool Chassis::get_auto_run_is_finished()
{
    return auto_run_is_finished;
}