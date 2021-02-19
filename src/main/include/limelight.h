#pragma once

#include <string>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

using namespace nt;


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
public:
    /**
     * Construct the class in the robot's init phase
     * @param tableName The Limelight's NetworkTables name (defaults to "limelight")
     */
    Limelight(std::string tableName = "limelight") {
        limelight = NetworkTableInstance::GetDefault().GetTable(tableName);
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
};
