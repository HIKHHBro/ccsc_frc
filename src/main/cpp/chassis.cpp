/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include "chassis.h"
#include "Math.h"
#include "frc/shuffleboard/Shuffleboard.h"
#include <vector> 
Chassis::Chassis(int can_id):MyThread(20000)
{
    try
    {
        ahrs = new AHRS(SPI::Port::kMXP);//陀螺仪
        limelight = new Limelight();
        motor_init(can_id);//电机初始化
        set_reference(CAR);//坐标系设置
        if(check_gyro())
        {
            world_angle = ahrs->GetYaw();
        }
        set_dia(76.2);
        set_reduction_ratiop(14,50,16,48);
        std::thread thr(std::bind(&Chassis::pid_loop,this));
        this->pid_thread = std::move(thr);
        this->pid_thread.detach();
        for(int i = 0;i<3;i++)
        {
            auto_run_map_pid[i] = new frc2::PIDController(pos_loop_kp[i],pos_loop_ki[i],pos_loop_kd[i]);
        }
        rampf[0] = new RampFunction(cahssis_acc[0]);//2000mm/s^0.5 * 0.02 = 40
        rampf[1] = new RampFunction(cahssis_acc[1]);//2000mm/s^2 * 0.02 = 40
        rampf[2] = new RampFunction(cahssis_acc[2]);//2rad/s^2 *0.02 = 0.04
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
    for(int i = 0;i<3;i++)
    {
    delete rampf[i];
    }
}

///< 底盘运动模型正解 单位 每100ms所转的编码器线数
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


///< 手动模式 vx:mm/s  vy:mm/s   vz:rad/s
void Chassis::rc_run(float vx,float vy,float vz)
{

    target_vel[0] = mms_to_enc100ms(vx);
    target_vel[1] = mms_to_enc100ms(vy);
    target_vel[2] = mms_to_enc100ms(vz);
    motion_model(target_vel[0],target_vel[1],target_vel[2]);
    for(int i=0;i<M_ALL;i++)
    {
        motor_pid[i]->pid_set(speed[i]);
    }
    milemter();
}

///<< 里程计计算
//TODO: 世界坐标系没有成功
bool Chassis::milemter()
{
    float w = 0;
    updata_series();
    if(check_gyro())
    {
        milemeter[z] = get_gyro();
    }
    else
    {
        milemeter[z] =  (enc_to_mm(wheel_s[M1]) +   enc_to_mm(wheel_s[M2]) + \
                  enc_to_mm(wheel_s[M3]) +   enc_to_mm(wheel_s[M4]))/3.355556/4.0;
    }
                       
    milemeter[x]  =   sin(DEG_TO_RAD(wheel_theta + w)) * enc_to_mm(wheel_s[M1]) \
                    + sin(DEG_TO_RAD(wheel_theta - w)) * enc_to_mm(wheel_s[M2]) \
                    - sin(DEG_TO_RAD(wheel_theta + w)) * enc_to_mm(wheel_s[M3]) \
                    - sin(DEG_TO_RAD(wheel_theta - w)) * enc_to_mm(wheel_s[M4]);
    milemeter[y]  = - cos(DEG_TO_RAD(wheel_theta + w)) * enc_to_mm(wheel_s[M1]) \
                    + cos(DEG_TO_RAD(wheel_theta - w)) * enc_to_mm(wheel_s[M2]) \
                    + cos(DEG_TO_RAD(wheel_theta + w)) * enc_to_mm(wheel_s[M3]) \
                    - cos(DEG_TO_RAD(wheel_theta - w)) * enc_to_mm(wheel_s[M4]);
    return true;
}

///<位置控制
bool Chassis::to_position(int point)
{

    // float milemeter[3];
    // milemeter[0] = milemeter[0];
    // milemeter[1] = milemeter[1];
    // milemeter[2] = milemeter[2];
    if((point ==1) && abs(milemeter[0]) <2000)
    {
        vision_x_distance = limelight->get_x();
        if(abs(vision_x_distance) <1000)
        {
            is_used_vision = true;
        }
        else is_used_vision = false;
    }
    else is_used_vision = false;
    frc::SmartDashboard::PutNumber("is_used_vision",is_used_vision);
    frc::SmartDashboard::PutNumber("vision_x_distance",vision_x_distance);
     frc::SmartDashboard::PutNumber("map3",map[2][0]);
     frc::SmartDashboard::PutNumber("map4",map[3][0]);
     frc::SmartDashboard::PutNumber("milemeter[0]",milemeter[0]);

    if(point < map_len)
    {
        try
        {   
            
            y_pos_error = abs(get_position_error(map[point][y],milemeter[y]));
            x_pos_error = abs(get_position_error(map[point][x],milemeter[x]));
            if(is_used_vision && (abs(x_pos_error) <500))
            {
               x_pos_error  = vision_x_distance;
            }
            frc::SmartDashboard::PutNumber("x_pos_error",x_pos_error);
            frc::SmartDashboard::PutNumber("y_pos_error",y_pos_error);
            frc::SmartDashboard::PutNumber("is_arrived_pos_error[x][point])",is_arrived_pos_error[x][point]);
            x_v_error = abs(motor[0]->GetSelectedSensorVelocity());
            y_v_error = abs(motor[2]->GetSelectedSensorVelocity());
            frc::SmartDashboard::PutNumber("y_v_error",y_v_error);
            if((abs(y_pos_error) < is_arrived_pos_error[y][point]) && (abs(x_pos_error) < is_arrived_pos_error[x][point]))
            {
                if((abs(x_v_error) < is_arrived_vel_error[x][point]) && (abs(y_v_error) < is_arrived_vel_error[y][point]))
                {
                    chassis_dis();
                    rc_run(0,0,0);
                    return true;
                }
                else
                {
                    if(is_used_vision)
                    {
                        auto_output[0] = auto_run_map_pid[0]->Calculate(-vision_x_distance,0);
                    }
                    else
                    {
                    auto_output[0] = auto_run_map_pid[0]->Calculate(milemeter[0],map[point][0]);
                    }
                    
                    auto_output[1] = auto_run_map_pid[1]->Calculate(milemeter[1],map[point][1]);
                    auto_output[2] = auto_run_map_pid[2]->Calculate(milemeter[2],0);

                    auto_output[0] = limit(auto_output[x],-map[point][2],map[point][2]);
                    auto_output[1] = limit(auto_output[y],-map[point][2],map[point][2]);
                
                    auto_output[0] = rampf[0]->cal_speed(auto_output[0]);
                    // auto_output[1] = rampf[1]->cal_speed(auto_output[1]);
                    // auto_output[2] = rampf[2]->cal_speed(auto_output[2]);
                    
                    rc_run(auto_output[x],auto_output[y],auto_output[z]);
                }

            }
            else
            {
                if(is_used_vision)
                {
                    auto_output[0] = auto_run_map_pid[0]->Calculate(-vision_x_distance,0);
                }
                else
                {
                auto_output[0] = auto_run_map_pid[0]->Calculate(milemeter[0],map[point][0]);
                }
                // auto_output[0] = auto_run_map_pid[0]->Calculate(milemeter[0],map[point][0]);
                auto_output[1] = auto_run_map_pid[1]->Calculate(milemeter[1],map[point][1]);
                auto_output[2] = auto_run_map_pid[2]->Calculate(milemeter[2],0);
                auto_output[0] = limit(auto_output[x],-map[point][2],map[point][2]);
                auto_output[1] = limit(auto_output[y],-map[point][2],map[point][2]);
                auto_output[0] = rampf[0]->cal_speed(auto_output[0]);
                auto_output[1] = rampf[1]->cal_speed(auto_output[1]);
                auto_output[2] = rampf[2]->cal_speed(auto_output[2]);
                rc_run(auto_output[x],auto_output[y],auto_output[z]);
            }
            frc::SmartDashboard::PutNumber("auto_output[0]",auto_output[0]);
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
    }
    return false;
}

///< 设置坐标系 
void Chassis::set_reference(Reference ref)
{
    reference = ref;
}
///<更新和累计编码器值
void Chassis::updata_series(void)
{
    for (int i = 0;i<M_ALL;i++)
    {
        wheel_s[i] = motor[i]->GetSelectedSensorPosition();
    }
}
///<重设编码器值
void Chassis::set_series()
{
    auto_run_is_finished = false;
    for(int i = 0;i<M_ALL;i++)
        motor[i]->SetSelectedSensorPosition(0, 0, 10);
}

//DONE: 完成测试
///< 开启自动模式
void Chassis::start_auto_run(void)
{
    if(!auto_run_is_finished)
    {
        start_detach();
    }
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
        motor[i]->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);
        motor[i]->SetNeutralMode(NeutralMode::Brake);
        /* set the peak and nominal outputs */
        motor[i]->ConfigNominalOutputForward(0, 10);
        motor[i]->ConfigNominalOutputReverse(0, 10);
        motor[i]->ConfigPeakOutputForward(1, 10);
        motor[i]->ConfigPeakOutputReverse(-1, 10);
        motor[i]->ConfigVelocityMeasurementPeriod(VelocityMeasPeriod::Period_100Ms,10);
        motor[i]->SetSelectedSensorPosition(0, 0, 10);
        motor[i]->ConfigNeutralDeadband(0,10);

        motor_pid[i] = new PIDControl(0.35,10,0,0.001,-max_enc_100ms,max_enc_100ms,MANUAL,DIRECT,20000);
        // motor_pid[i] = new PIDControl(1,23,0,0.001,-max_enc_100ms,max_enc_100ms,MANUAL,DIRECT,20000);
    }
}
///< 获取自动模式是否完成
bool Chassis::get_auto_run_is_finished()
{
    return auto_run_is_finished;
}
///< 线程函数的重写
void Chassis::run()
{
    auto_run_is_finished =false;
    chassis_dis();
    while (!isInterrupted())
    {
        milemter();
        if(auto_point != 0)
        {
            arrived_point = to_position(auto_point);
        }
        else
        {
            chassis_dis();
            interrupt();
            auto_run_is_finished = true;
        }

        thread_sleep();
    }
    //线程结束，数据清除
    interrupt();
    chassis_dis();
    auto_point = 0;
}
///< 底盘pid计算线程
//待测试
void Chassis::pid_loop()
{
    is_interript_pid = false;
    std::cout<<"pid"<<std::endl;
    while (true)
    {
        for(int i = 0;i<M_ALL;i++)
        {
            speed_output_per[i] = motor_pid[i]->PIDCompute(motor[i]->GetSelectedSensorVelocity());
            speed_output_per[i] = enc100ms_to_per(speed_output_per[i]);
            speed_output_per[i] = limit(speed_output_per[i],-1.0,1.0);
            motor[i]->Set(ControlMode::PercentOutput,speed_output_per[i]);
        }
        // timer_sleep(0,1000);
        usleep(1000);
    }
    
}

void Chassis::chassis_dis()
{
    target_vel[0] = 0;
    target_vel[1] = 0;
    target_vel[2] = 0;
    speed[0] = 0;
    speed[1] = 0;
    speed[2] = 0;
    speed[2] = 0;
    speed_output_per[0] = 0;
    speed_output_per[1] = 0;
    speed_output_per[2] = 0;
    speed_output_per[3] = 0;
    y_pos_error= 0;
    x_pos_error= 0;
    x_v_error= 0;
    y_v_error= 0;
    auto_output[0] = 0;
    auto_output[1] = 0;
    auto_output[2] = 0;
    for(int i = 0;i<M_ALL;i++)
    {
        motor_pid[i]->clear_output();
    }
    if(check_gyro())
        init_angle = ahrs->GetYaw();
}
///< 获取机体坐标的角度
float Chassis::get_gyro()
{
  return (ahrs->GetYaw() - init_angle);
}
// ///< 角度控制
// //TODO: 注意角度单位，和角度限制
// void Chassis::angle_control(float angle)
// {
//     auto_output[2] =limit(angle,-2.0,2.0);
//     rc_run(0,0,auto_output[2]);
// }
///< 底盘进入使能模式后清除所有
void Chassis::clear()
{
  interrupt();
  chassis_dis();
  set_series();
}
///< 设置底盘自动阶段运行的点
void Chassis::set_auto_point(int point)
{
    if(point < map_len)
    {
       auto_point = point;
    }
}
///< 获取底盘自动阶段运行当前的点
int Chassis::get_auto_point() 
{
    return auto_point;
}
///< 是否到达指定点
bool Chassis::is_arrived_point()
{
    return arrived_point;
}
///< 地图目标点设置
void Chassis::set_map_test_display()
{
    frc::SmartDashboard::PutNumber("point numb",map_len);
    for(int i = 0;i<map_len;i++)
    {
        std::string temp_namex = "point x" + std::to_string (i);
        std::string temp_namey = "point y" + std::to_string (i);
        std::string temp_namev = "point v" + std::to_string (i);

        frc::SmartDashboard::PutNumber("temp_namex",map[i][0]);
        frc::SmartDashboard::PutNumber("temp_namey",map[i][1]);
        frc::SmartDashboard::PutNumber("temp_namev",map[i][2]);
    }
}
void Chassis::set_map_test()
{
    float numb = frc::SmartDashboard::GetNumber("point numb",map_len);
    if(numb != map_len)
    {
        if(numb > 0 && numb <10)
            map_len = numb;
    }
    for(int i = 0;i<map_len;i++)
    {
        StringRef temp_namex = "point x" + std::to_string (i);
        StringRef temp_namey = "point y" + std::to_string (i);
        StringRef temp_namev = "point v" + std::to_string (i);
        map[i][0] = get_number(temp_namex,map[i][0],-2000.0,2000.0);
        map[i][1] = get_number(temp_namey,map[i][1],0.0,7000.0);
        map[i][2] = get_number(temp_namev,map[i][2],0.0,1300.0);
    }
}
#ifdef CHASSIS_DEBUG
void Chassis::display()
{ 
    // // frc::Shuffleboard::GetTab("Example tab").Add(gyro);
    // frc::SmartDashboard::PutNumber("速度环Kp",speed_loop_kp); 
    // frc::SmartDashboard::PutNumber("速度环Ki",speed_loop_ki); 

    // frc::SmartDashboard::PutNumber("位置环Kp",pos_loop_kp[1]); 
    // frc::SmartDashboard::PutNumber("位置环Ki",pos_loop_ki[1]); 
    // frc::SmartDashboard::PutNumber("位置环Ki",pos_loop_kd[1]);

    // frc::SmartDashboard::PutNumber("cahssis_acc[0]",cahssis_acc[0]);
    // frc::SmartDashboard::PutNumber("cahssis_acc[1]",cahssis_acc[1]);
    // frc::SmartDashboard::PutNumber("cahssis_acc[2]",cahssis_acc[2]);

    
}
void Chassis::debug()
{

    // float Get1 = frc::SmartDashboard::GetNumber("速度环Kp",speed_loop_kp);
    // if(Get1 != speed_loop_kp)
    // {
    //     speed_loop_kp = Get1;
    //     for(int i = 0;i<M_ALL;i++)
    //         motor_pid[i]->PIDTuningsSet(speed_loop_kp,speed_loop_ki,0);
    // };

    // float Get2 = frc::SmartDashboard::GetNumber("速度环Ki",speed_loop_ki);
    // if(Get2 != speed_loop_ki) 
    // {
    //     speed_loop_ki = Get2;
    //     for(int i = 0;i<M_ALL;i++)
    //         motor_pid[i]->PIDTuningsSet(speed_loop_kp,speed_loop_ki,0);
    // }

    // float Get3 = frc::SmartDashboard::GetNumber("位置环Kp",pos_loop_kp[1]);
    // if(Get3 != pos_loop_kp[1])
    // {
    //     pos_loop_kp[1] = Get3;
    //     for(int i = 0;i<2;i++)
    //         auto_run_map_pid[i]->SetP(pos_loop_kp[1]);
    // }

    // float Get4 = frc::SmartDashboard::GetNumber("位置环Ki",pos_loop_ki[1]);
    // if(Get4 != pos_loop_ki[1])
    // {
    //     pos_loop_ki[1] = Get4;
    //     for(int i = 0;i<2;i++)
    //         auto_run_map_pid[i]->SetI(pos_loop_ki[1]);
    // }

    // float Get5 = frc::SmartDashboard::GetNumber("位置环Ki",pos_loop_kd[1]);
    // if(Get5 != pos_loop_kd[1])
    // {
    //     pos_loop_kd[1] = Get5;
    //     for(int i = 0;i<2;i++)
    //         auto_run_map_pid[i]->SetD(pos_loop_kd[1]);
    // }

    // map[1][0] = get_number("目标点x mm",map[1][0],0.0,10000.0);
    // map[1][1] = get_number("目标点y mm",map[1][1],0.0,10000.0);
    // map[1][2] = get_number("目标速度mm/s",map[1][2],0.0,10000.0);

    // cahssis_acc[0] = get_number("cahssis_acc[0]",cahssis_acc[0],0.0,1000.0);
    // cahssis_acc[1] = get_number("cahssis_acc[1]",cahssis_acc[1],0.0,1000.0);
    // cahssis_acc[2] = get_number("cahssis_acc[2]",cahssis_acc[2],0.0,2.0);

    // test_angle = get_number("test_angle",test_angle,-180.0,180.0);

    // pos_loop_kp[2] = get_number("pos_loop_kp[2]",pos_loop_kp[2],-0.05,0.05);
    // auto_run_map_pid[2]->SetP(pos_loop_kp[2]);

    // pos_loop_kd[2] = get_number("pos_loop_kd[2]",pos_loop_kd[2],-0.05,0.05);
    // auto_run_map_pid[2]->SetD(pos_loop_kd[2]);

    // frc::SmartDashboard::PutNumber("X 速度enc/100ms",target_vel[x]); //速度范围-6380~6380
    // frc::SmartDashboard::PutNumber("Y 速度enc/100ms",target_vel[y]); 
    // frc::SmartDashboard::PutNumber("Z 速度enc/100ms",target_vel[z]); 

    // frc::SmartDashboard::PutNumber("speed[0]",speed[0]); 
    // frc::SmartDashboard::PutNumber("speed[1]",speed[1]); 
    // frc::SmartDashboard::PutNumber("speed[2]",speed[2]); 
    // frc::SmartDashboard::PutNumber("speed[3]",speed[3]); 

    // frc::SmartDashboard::PutNumber("坐标系",reference); 
    // frc::SmartDashboard::PutNumber("减速比",reduction_ratiop); 


    // frc::SmartDashboard::PutNumber("位置X ms",milemeter[x]);
    // frc::SmartDashboard::PutNumber("位置Y ms",milemeter[y]);
    // frc::SmartDashboard::PutNumber("角度 ms",milemeter[z]);

    // frc::SmartDashboard::PutNumber("X位置误差",x_pos_error);
    // frc::SmartDashboard::PutNumber("y位置误差",y_pos_error);
    // frc::SmartDashboard::PutNumber("X速度误差",x_v_error);
    // frc::SmartDashboard::PutNumber("y速度误差",y_v_error);


    // frc::SmartDashboard::PutNumber("X位置到达阈值",is_arrived_pos_error[y]);
    // frc::SmartDashboard::PutNumber("y位置到达阈值",is_arrived_pos_error[x]);
    // frc::SmartDashboard::PutNumber("X位置到达阈值",is_arrived_vel_error[x]);
    // frc::SmartDashboard::PutNumber("y位置到达阈值",is_arrived_vel_error[y]);



    // frc::SmartDashboard::PutNumber("y位置环输出",auto_output[y]);
    // frc::SmartDashboard::PutNumber("X位置环输出",auto_output[x]);
    // frc::SmartDashboard::PutNumber("角度位置环输出",auto_output[z]);

    
    // frc::SmartDashboard::PutNumber("电机1速度环输出",motor_pid[0]->PIDOutputGet());
    // frc::SmartDashboard::PutNumber("电机2速度环输出",motor_pid[1]->PIDOutputGet());
    // frc::SmartDashboard::PutNumber("电机3速度环输出",motor_pid[2]->PIDOutputGet());
    // frc::SmartDashboard::PutNumber("电机4速度环输出",motor_pid[3]->PIDOutputGet());

    
    // frc::SmartDashboard::PutNumber("电机1速度环百分比输出",speed_output_per[0]);
    // frc::SmartDashboard::PutNumber("电机2速度环百分比输出",speed_output_per[1]);
    // frc::SmartDashboard::PutNumber("电机3速度环百分比输出",speed_output_per[2]);
    // frc::SmartDashboard::PutNumber("电机4速度环百分比输出",speed_output_per[3]);

    // frc::SmartDashboard::PutNumber("rampf[0]",rampf[0]->get_last_data());
    // frc::SmartDashboard::PutNumber("rampf[1]",rampf[1]->get_last_data());
    // frc::SmartDashboard::PutNumber("rampf[2]",rampf[2]->get_last_data());

    // frc::SmartDashboard::PutNumber("test_output[0]",motor_pid[0]->test_output);
    // frc::SmartDashboard::PutNumber("motor[0] vel",motor[0]->GetSelectedSensorVelocity()); 
    frc::SmartDashboard::PutNumber("auto_point",auto_point); 

    

    thread_debug();

    // display();
    
}
#endif

