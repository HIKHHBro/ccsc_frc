
#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    MANUAL,
    AUTOMATIC
}PIDiTermMode;

typedef enum
{
    DIRECT,
    REVERSE
}PIDDirection;

class PIDControl
{
public:
double test_output = 0;
    // 
    // Constructor
    // Description:
    //      Initializes the PIDControl instantiation. This should be called at 
    //      least once before any other PID functions are called on the 
    //      instantiation.
    // Parameters:
    //      kp - Positive P gain constant value.
    //      ki - Positive I gain constant value.
    //      kd - Positive D gain constant value.
    //      sampleTimeSeconds - Interval in seconds on which PIDCompute will be 
    //          called.
    //      minOutput - Constrain PID output to this minimum value.
    //      maxOutput - Constrain PID output to this maximum value.
    //      mode - Tells how the controller should respond if the user has 
    //          taken over manual control or not.
    //          MANUAL:    PID controller is off. User can manually control the 
    //                     output.
    //          AUTOMATIC: PID controller is on. PID controller controls the 
    //                     output.
    //      controllerDirection - The sense of direction of the controller
    //          DIRECT:  A positive setpoint gives a positive output.
    //          REVERSE: A positive setpoint gives a negative output.
    // Returns:
    //      Nothing.
    // 
    PIDControl(float kp, float ki, float kd, float sampleTimeSeconds, 
                float minOutput, float maxOutput, PIDiTermMode mode, 
                PIDDirection controllerDirection,float max_speed);     	
    
    // 
    // PID Compute
    // Description:
    //      Should be called on a regular interval defined by sampleTimeSeconds.
    //      Typically, PIDSetpointSet and PIDInputSet should be called 
    //      immediately before PIDCompute.
    // Parameters:
    //      None.
    // Returns:
    //      True if in AUTOMATIC. False if in MANUAL.
    //                     
    bool PIDCompute(); 
    float PIDCompute(float in);
    void pid_set(float point);
    void clear_output();
    
    // 
    // PID Mode Set
    // Description:
    //      Sets the PID controller to a new mode. Tells how the controller 
    //      should respond if the user has taken over manual control or not.
    // Parameters:
    //      mode - 
    //          MANUAL:    PID controller is off. User can manually control the 
    //                     output.
    //          AUTOMATIC: PID controller is on. PID controller controls the 
    //                     output.
    // Returns:
    //      Nothing.
    //              
    void pid_iterm_mode(PIDiTermMode mode);
    
    // 
    // PID Output Limits Set
    // Description:
    //      Sets the new output limits. The new limits are applied to the PID
    //      immediately.
    // Parameters:
    //      min - Constrain PID output to this minimum value.
    //      max - Constrain PID output to this maximum value.
    // Returns:
    //      Nothing.
    // 
    void PIDOutputLimitsSet(float min, float max);
    
    // 
    // PID Tunings Set
    // Description:
    //      Sets the new gain constant values.
    // Parameters:
    //      kp - Positive P gain constant value.
    //      ki - Positive I gain constant value.
    //      kd - Positive D gain constant value.
    // Returns:
    //      Nothing.
    // 
    void PIDTuningsSet(float kp, float ki, float kd);
    
    // 
    // PID Tuning Gain Constant P Set
    // Description:
    //      Sets the proportional gain constant value.
    // Parameters:
    //      kp - Positive P gain constant value.
    // Returns:
    //      Nothing.
    // 
    void PIDTuningKpSet(float kp);
    
    // 
    // PID Tuning Gain Constant I Set
    // Description:
    //      Sets the proportional gain constant value.
    // Parameters:
    //      ki - Positive I gain constant value.
    // Returns:
    //      Nothing.
    // 
    void PIDTuningKiSet(float ki);
    
    // 
    // PID Tuning Gain Constant D Set
    // Description:
    //      Sets the proportional gain constant value.
    // Parameters:
    //      kd - Positive D gain constant value.
    // Returns:
    //      Nothing.
    // 
    void PIDTuningKdSet(float kd);
    
    // 
    // PID Controller Direction Set
    // Description:
    //      Sets the new controller direction.
    // Parameters:
    //      controllerDirection - The sense of direction of the controller
    //          DIRECT:  A positive setpoint gives a positive output
    //          REVERSE: A positive setpoint gives a negative output
    // Returns:
    //      Nothing.
    // 
    void PIDControllerDirectionSet(PIDDirection controllerDirection);
    
    // 
    // PID Sample Time Set
    // Description:
    //      Sets the new sampling time (in seconds).
    // Parameters:
    //      sampleTimeSeconds - Interval in seconds on which PIDCompute will be 
    //          called.
    // Returns:
    //      Nothing.
    // 
    void PIDSampleTimeSet(float sampleTimeSeconds);
    // 
    // PID Setpoint Set
    // Description:
    //      Alters the setpoint the PID controller will try to achieve.
    // Parameters:
    //      setpoint - The desired setpoint the PID controller will try to 
    //          obtain.
    // Returns:
    //      Nothing.
    // 
    inline void PIDSetpointSet(float setpoint) { this->setpoint = setpoint; }
    
    // 
    // PID Input Set
    // Description:
    //      Should be called before calling PIDCompute so the PID controller 
    //      will have an updated input value to work with.
    // Parameters:
    //      input - The value the controller will work with.
    // Returns:
    //      Nothing.
    // 
    inline void PIDInputSet(float input) { this->input = input; }
    
    // 
    // PID Output Get
    // Description:
    //      Typically, this function is called after PIDCompute in order to
    //      retrieve the output of the controller.
    // Parameters:
    //      None.
    // Returns:
    //      The output of the specific PID controller.
    // 
    inline float PIDOutputGet() { return this->output; }
    
    // 
    // PID Proportional Gain Constant Get
    // Description:
    //      Returns the proportional gain constant value the particular
    //      controller is set to.
    // Parameters:
    //      None.
    // Returns:
    //      The proportional gain constant.
    // 
    inline float PIDKpGet() { return this->dispKp; }						  
    
    // 
    // PID Integral Gain Constant Get
    // Description:
    //      Returns the integral gain constant value the particular
    //      controller is set to.
    // Parameters:
    //      None.
    // Returns:
    //      The integral gain constant.
    // 
    inline float PIDKiGet() { return this->dispKi; }						  
    
    // 
    // PID Derivative Gain Constant Get
    // Description:
    //      Returns the derivative gain constant value the particular
    //      controller is set to.
    // Parameters:
    //      None.
    // Returns:
    //      The derivative gain constant.
    // 
    inline float PIDKdGet() { return this->dispKd; }						  
    
    // 
    // PID Mode Get
    // Description:
    //      Returns the mode the particular controller is set to.
    // Parameters:
    //      None.
    // Returns:
    //      MANUAL or AUTOMATIC depending on what the user set the 
    //      controller to.
    // 
    inline PIDiTermMode PIDModeGet() { return this->mode; }						  
    
    // 
    // PID Direction Get
    // Description:
    //      Returns the direction the particular controller is set to.
    // Parameters:
    //      None.
    // Returns:
    //      DIRECT or REVERSE depending on what the user set the
    //      controller to.
    // 
    inline PIDDirection PIDDirectionGet() { return this->controllerDirection; }
    enum PIDMode{Abs,Inc};
private:
    // 
    // Input to the PID Controller
    // 
    float input = 0;
    
    // 
    // Previous input to the PID Controller
    // 
    float lastInput = 0;
    
    // 
    // Output of the PID Controller
    // 
    double output = 0;
    
    // 
    // Gain constant values that were passed by the user
    // These are for display purposes
    // 
    float dispKp = 0;
    float dispKi = 0;
    float dispKd = 0;
    
    // 
    // Gain constant values that the controller alters for
    // its own use
    // 
    float alteredKp = 0;
    float alteredKi = 0;
    float alteredKd = 0;
    
    // 
    // The Integral Term
    // 
    float iTerm = 0;
    
    // 
    // The interval (in seconds) on which the PID controller
    // will be called
    // 
    float sampleTime = 0;
    
    // 
    // The values that the output will be constrained to
    // 
    float outMin = 0;
    float outMax = 0;
    
    // 
    // The user chosen operating point
    // 
    float setpoint = 0;
    
    // 
    // The sense of direction of the controller
    // DIRECT:  A positive setpoint gives a positive output
    // REVERSE: A positive setpoint gives a negative output
    // 
    PIDDirection controllerDirection = DIRECT;
    
    // 
    // Tells how the controller should respond if the user has
    // taken over manual control or not
    // MANUAL:    PID controller is off.
    // AUTOMATIC: PID controller is on.
    // 
    PIDiTermMode mode = MANUAL;
    float error = 0;
    float last_error = 0;
    float pre_last_error = 0;
    float max_speed = 0;
};

#endif  // PID_CONTROLLER_H