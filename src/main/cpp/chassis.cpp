/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include "chassis.h"
#include "Math.h"
#include "my_thread.h"
Chassis::Chassis(int can_id):MyThread(20000)
{
    try
    {
        ahrs = new AHRS(SPI::Port::kMXP);//陀螺仪
        motor_init(can_id);//电机初始化
        set_reference(CAR);//坐标系设置
        set_series(0);
        if(check_gyro())
        {
            world_angle = ahrs->GetYaw();
        }
        set_dia(76.2);
        set_reduction_ratiop(22,42,14,50);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    
}
Chassis::~Chassis()
{
    delete ahrs;
    delete auto_run_map_pid[0];
    delete auto_run_map_pid[1];
    delete auto_run_map_pid[2];
    delete motor[0];
    delete motor[1];
    delete motor[2];
    delete motor[3];


}

//TODO: 待测试  1、机体坐标系 2、世界坐标系 //单位 mm/s
///< 底盘运动模型正解
void Chassis::motion_model(float vx,float vy,float vz)
{
    float w = 0;
    float x = vx; 
    float y = vy;
    float z = vz; 
    if(reference == WORLD)
    {
        if(check_gyro())
        {
            // w = ahrs->GetYaw();
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
    
    float max = speed[M1];
    float max_enc = rpm_to_enc_100ms(max_rpm);
    for(int i = 1;i<M_ALL;i++)
    {
        if(abs(speed[i]) > abs(max))
        {
            max = speed[i];
        }
    }
    if(abs(max) > max_enc)
    {
        float coep = max_enc/max;
        for(int i = 0;i<M_ALL;i++)
        {
            speed[i] = speed[i] * coep;
        }
    }
}
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
//速度 mm/s
//待测试
void Chassis::rc_run(float vx,float vy,float vz)
{
    target_vel[0] = mms_to_enc100ms(vx);
    target_vel[1] = mms_to_enc100ms(vy);
    target_vel[2] = mms_to_enc100ms(vz);
    motion_model(target_vel[0],target_vel[1],target_vel[2]);
    for(int i=0;i<M_ALL;i++)
    {
        // std::cout<<"s1 = "<<ramp_func[i]->set(speed[i]);
        motor_pid[i]->pid_set(speed[i]);
        motor_pid[i]->PIDCompute(motor[i]->GetSelectedSensorVelocity());
        motor[i]->Set(ControlMode::PercentOutput,motor_pid[i]->get_out());
    }
}


//TODO:待测试
///<< 里程计计算
//TODO: 世界坐标系没有成功
float tmp_angle = 0;
bool Chassis::milemter()
{
    // if(check_gyro())
    // {
    //     // float temp_w = ahrs->GetYaw(); 
    //          float w = 0;
    //     updata_series();
    //     milemeter[z] =  (series_to_mm(wheel_s[M1]) +   series_to_mm(wheel_s[M2]) + \
    //                     series_to_mm(wheel_s[M3]) +   series_to_mm(wheel_s[M4]))/3.355556/4.0;
    //     w = milemeter[z] + world_angle;                
    //     milemeter[x]  =   sin(DEG_TO_RAD(wheel_theta + w)) * series_to_mm(wheel_s[M1]) \
    //                     + sin(DEG_TO_RAD(wheel_theta - w)) * series_to_mm(wheel_s[M2]) \
    //                     - sin(DEG_TO_RAD(wheel_theta + w)) * series_to_mm(wheel_s[M3]) \
    //                     - sin(DEG_TO_RAD(wheel_theta - w)) * series_to_mm(wheel_s[M4]);
    //     milemeter[y]  = - cos(DEG_TO_RAD(wheel_theta + w)) * series_to_mm(wheel_s[M1]) \
    //                     + cos(DEG_TO_RAD(wheel_theta - w)) * series_to_mm(wheel_s[M2]) \
    //                     + cos(DEG_TO_RAD(wheel_theta + w)) * series_to_mm(wheel_s[M3]) \
    //                     - cos(DEG_TO_RAD(wheel_theta - w)) * series_to_mm(wheel_s[M4]);
    //     std::cout<<" milemeter[x]= "<< milemeter[x]\
    //              <<" milemeter[y]= "<< milemeter[y]\
    //              <<" w= "<< w<<std::endl;
    //     return true;
    // }
    // else 
    // {
    //     std::cout<<"无陀螺仪"<<std::endl;
    //     return false;
    // }
   
return false;
}

///<位置控制
bool Chassis::to_position(float x,float y,float w)
{
    return true;
}

///< 设置坐标系 
void Chassis::set_reference(Reference ref)
{
    reference = ref;
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


}

//DONE: 完成测试
///< 开启自动模式
void Chassis::start_auto_run(void)
{
    start_detach();
}
///< 退出自动模式
void Chassis::exit_auto_run()
{
    interrupt();
}

///< 电机初始化
void Chassis::motor_init(int id)
{
    
    for(int i=0;i<M_ALL;i++)
    {
        motor[i] = new TalonFX(i+id);  
        /* Factory default hardware to prevent unexpected behavior */
        motor[i]->ConfigFactoryDefault();
        /* first choose the sensor */
        motor[i]->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 0);
        // motor[i]->SetSensorPhase(true);
        motor[i]->SetNeutralMode(NeutralMode::Brake);
        motor[i]->ConfigNeutralDeadband(0);
        /* set the peak and nominal outputs */
        motor[i]->ConfigNominalOutputForward(0, 0);
        motor[i]->ConfigNominalOutputReverse(0, 0);
        motor[i]->ConfigPeakOutputForward(1, 0);
        motor[i]->ConfigPeakOutputReverse(-1, 0);
        motor[i]->ConfigVelocityMeasurementPeriod(VelocityMeasPeriod::Period_100Ms,0);//TODO:改成1ms
        motor[i]->ConfigVelocityMeasurementWindow(1,0);
        motor_pid[i] = new PIDControl(0.05,0.05,0,0.001,-1,1,MANUAL,DIRECT,20000,PIDControl::Inc);
    }
}
//TODO: 待测试
///< 获取自动模式是否完成
bool Chassis::get_auto_run_is_finished()
{
    return auto_run_is_finished;
}

///< 线程函数的重写
void Chassis::run()
{

    float output[3];
    auto_run_map_pid[x] = new frc2::PIDController(5.0,0.02,5.0);
    auto_run_map_pid[y] = new frc2::PIDController(5.0,0.02,5.0);
    auto_run_map_pid[z] = new frc2::PIDController(5.0,0.02,5.0);
    auto_run_is_finished =false;
    for(int i =1;(i<map_len)||!isInterrupted();i++)
    {
        milemter();
        try
        {
            if(abs(auto_run_map_pid[y]->GetPositionError()) > abs(map[i][y-1] - map[i][y])/2)
            {
                i--;
            }
            for(int j = 0;j<3;j++)
            {
                output[j] = auto_run_map_pid[j]->Calculate(milemeter[j],map[i][j]);
                
            }
            rc_run(output[x],output[y],output[z]);
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
        thread_sleep();
        std::cout<< "this auto run"<<"i="<<i<<std::endl;
    }
    auto_run_is_finished = true;
}



#ifdef CHASSIS_DEBUG
void Chassis::display()
{ 
}
void Chassis::debug()
{

    frc::SmartDashboard::PutNumber("X 速度enc/100ms",target_vel[x]); //速度范围-6380~6380
    frc::SmartDashboard::PutNumber("Y 速度enc/100ms",target_vel[y]); 
    frc::SmartDashboard::PutNumber("Z 速度enc/100ms",target_vel[z]); 

    frc::SmartDashboard::PutNumber("右前轮enc/100ms",speed[0]); 
    frc::SmartDashboard::PutNumber("左前轮enc/100ms",speed[1]); 
    frc::SmartDashboard::PutNumber("左后轮enc/100ms",speed[2]); 
    frc::SmartDashboard::PutNumber("右后轮enc/100ms",speed[3]); 


    frc::SmartDashboard::PutNumber("坐标系",reference); 
    frc::SmartDashboard::PutNumber("减速比",reduction_ratiop); 

}
#endif