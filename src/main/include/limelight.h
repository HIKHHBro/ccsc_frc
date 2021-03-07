#pragma once

#include <string>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/SerialPort.h>
#include <frc/smartdashboard/smartdashboard.h>
using namespace nt;
using namespace frc;

/// Modes for the LEDs on the Limelight
enum LEDmode {
    pipeline,   ///< Let the pipeline decide
    off,        ///< Force off
    blink,      ///< Force blink
    on          ///< Force on
};

/// Modes for the camera on the Limelight
enum CamMode {
    vision,     ///< Use the limelight for vision processing
    driver      ///< Use the limelight as a driver camera
};

/**
 * NetworkTables wrapper for the Limelight
 */
class Limelight {
private:
    std::shared_ptr<NetworkTable> limelight;
    float pitch_max_angle = 23;
    float camtran[6] = {0};
    SerialPort *ultrasonic;
    char buffer[10] = {0};
    int distance = 0;
public:
    /**
     * Construct the class in the robot's init phase
     * @param tableName The Limelight's NetworkTables name (defaults to "limelight")
     */
    Limelight(std::string tableName = "limelight",float angle = 23) {
        limelight = NetworkTableInstance::GetDefault().GetTable(tableName);
        pitch_max_angle = angle;
        ultrasonic = new SerialPort(9600);
        ultrasonic->DisableTermination();
        // ultrasonic->SetFlowControl(SerialPort::kFlowControl_None);
    
        // ultrasonic->SetReadBufferSize(4);
    }

    /**
     * Returns the base Limelight NetworkTable
     * @returns The Limelight NetworkTable smart pointer
     */
    std::shared_ptr<NetworkTable> get() {
        return limelight;
    }

    /**
     * Tells you if a target is visible or not
     * @returns true if a target can be found, or false if a target cannot be found
     */
    bool hasTarget() {
        return limelight->GetBoolean("tv", false);
    }

    /**
     * Gets the X of the target
     * @returns The X of the target in a double
     */
    double getTargetX() {
        return limelight->GetNumber("tx", 0.0);
    }

    /**
     * Gets the Y of the target
     * @returns The Y of the target in a double
     */
    double getTargetY() {
        return limelight->GetNumber("ty", 0.0);
    }

    /**
     * Gets the area the target takes up of the Limelight
     * @returns Returns 1 for 100%, 0.5 for 50%, 0 for 0%, and so forth
     */
    double getTargetArea() {
        return limelight->GetNumber("ta", 0.0);
    }

    /**
     * Gets the skew of the target
     * @returns Returns the skew of the target from -90 to 0 degrees
     */
    double getTargetSkew() {
        return limelight->GetNumber("ts", 0.0);
    }

    /**
     * Gets the latency contribution of the limelight's pipeline
     * @returns Returns the latency in milleseconds, add 11 ms for image capture latency
     */
    double getPipelineLatency() {
        return limelight->GetNumber("tl", 0.0);
    }
    float* get_camtran(){
        // camtran = limelight->GetNumberArray("camtran", camtran);
        return camtran;
    }
    int tmp_angle = 0;
    ///< 获取发射补偿角度
    float get_pitch_angle(){
        
        updata_distance();
        if(distance >1000 && distance < 5000)
        {
            tmp_angle = 35 -0.0075 * distance;
            tmp_angle = (tmp_angle) > (23) ? (23) : (tmp_angle);
            tmp_angle = (tmp_angle) < (0) ? (0) : (tmp_angle);
            
        }
        return tmp_angle;
    }
    float get_x()
    {
        float x = -getTargetX();
        return x * 50;
    }
      


    /**
     * Sets the mode for the LEDs on the Limelight
     * @param mode The LEDmode to set the LED
     */
    void setLEDMode(LEDmode mode) {
        limelight->PutNumber("ledMode", mode);
    }

    /**
     * Sets the mode the camera is on
     * @param mode The CamMode to set the Camera
     */
    void setCamMode(CamMode mode) {
        limelight->PutNumber("camMode", 0.0);
    }

    /**
     * Sets the pipeline for the limelight to use
     * @param ID The pipeline ID (from 0-9) for the limelight to use
     */
    void setPipeline(int ID) {
        limelight->PutNumber("pipeline", ID);
    }

    ///< 计算校验和
    int cal_check_sum()
    {
        return ((buffer[0] + buffer[1] + buffer[2])&0x00FF);
    }
    ///< 更新超声波测距的距离
    void updata_distance()
    {
        memset(buffer,'\0',sizeof(buffer));
        ultrasonic->Read(buffer,4);
        if(buffer[0] == 0xFF && buffer[3] == cal_check_sum())
        {
            distance = (buffer[1]<<8) | (buffer[2]);
        }
        frc::SmartDashboard::PutNumber("ultr distance",distance);
    }
    ///< 测试超声波
    void test_ultrasonic()
    {

        frc::SmartDashboard::PutNumber("ultr buffer[0]",buffer[0]);
        frc::SmartDashboard::PutNumber("ultr buffer[1]",buffer[1]);
        frc::SmartDashboard::PutNumber("ultr buffer[2]",buffer[2]);
        frc::SmartDashboard::PutNumber("ultr buffer[3]",buffer[3]);
        frc::SmartDashboard::PutNumber("ultr distance",distance);
        memset(buffer,'\0',sizeof(buffer)); 
    }



};
