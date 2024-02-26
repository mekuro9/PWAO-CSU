
/*==============================================================
// Filename :       PIDController.hpp
// Version :        v1.0
// License :  
// Description :    Header file for pid controller  
//                  Based on Arduino PID Library - Version 1.1.1
//
// Modified by: Mrinal Magar , <magarmrinal@gmail.com>
// 
// Copyright (C) Brett Beauregard , <br3ttb@gmail.com>
// 
//                                 GPLv3 License
==============================================================*/

#ifndef PIDCONTROLLER_HPP
#define PIDCONTROLLER_HPP

#include <iostream>
#include "FeedbackController.hpp"
#include <Arduino.h>


enum class pid_controller_type{
        typePID,
        typePI
    };

template <typename T, typename U>
class PIDController : public FeedbackController<T, U> {

public:

    // Constructor
    PIDController(T*, T*, U*, float, float, float, pid_controller_type);
    
    PIDController(T*, T*, U*, float, float, pid_controller_type);
    // Deconstructor
    ~PIDController() = default;

    bool setup() override;
    void compute() override;
    void setControllerType(pid_controller_type); 
    void setSampleTime(float);
    bool setOutputLimits(U,U);


private:
    bool init();
    // Inputs and outputs of the controller block
    // which is the error and the controller output (which is u(t) input to the plant)
    // e(t) = r(t) - y(t) // where r(t) is the setpoint and y(t) is the plant state measured by sensors
    T *measuredVal_;
    T *setPoint_;
    U *controllerOutput_;
    // PID Controller gains, for proportional, derivative and integral control
    float kp, ki, kd;
    T iTerm, pTerm, dTerm;
    // Integral term uses an errorsum and derivate term needs error_current - error_last;
    float errorSum;
    auto measurementPrev;
    // Keeping track of lastTime and sampleTime
    unsigned long lastTime;

    // sample time in seconds
    float sampleTime;

    // derivative low pass filter time constant in seconds
    unsigned long tau;

    // Clamping the outputs
    U output_max, output_min;

    pid_controller_type controllerType;

};

#endif // PIDCONTROLLER_H


