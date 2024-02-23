/*==============================================================
// Filename :       PIDController.cpp
// Version :        v1.0
// License :  
// Description :    Implementation file of class PIDController.hpp
// Modified by: Mrinal Magar , <magarmrinal@gmail.com>
// 
// Copyright (C) Brett Beauregard , <br3ttb@gmail.com>
// 
//                                 GPLv3 License
==============================================================*/

#include "PIDController.hpp"

template <typename T, typename U>
PIDController<T, U>::PIDController(T *setPoint, T *measuredVal , U *controllerOutput, float kp, float ki, float kd, pid_controller_type type)
: setPoint_(setPoint), measuredVal_(measuredVal), controllerOutput_(controllerOutput), kp(kp), ki(ki), kd(kd), controllerType(type)
{
   measurementPrev = 0.0f;
   errorSum = 0.0f;
   sampleTime = 100; // 1 second is 1000 ms
   PIDController::setOutputLimits(0, 255); // Default output limits corresponding to arduino PWM limits
   lastTime = millis() - sampleTime;
}

// PI controller
template <typename T, typename U>
PIDController<T, U>::PIDController(T *setPoint, T *measuredVal, U *controllerOutput, float kp, float ki, pid_controller_type type)
: setPoint_(setPoint), measuredVal_(measuredVal), controllerOutput_(controllerOutput), kp(kp), ki(ki), kd(0.0f), controllerType(type) {
    measurementPrev = 0.0f;
    errorSum = 0.0f;
    sampleTime = 100; // Assuming 1 second is 1000 ms
    lastTime = millis() - sampleTime;
    setOutputLimits(0, 255); // Default output limits corresponding to Arduino PWM limits
}

template <typename T, typename U>
bool PIDController<T, U>::setup()
{  
   bool setup = init();
   if(!setup) return false;

   return true;

}

template <typename T, typename U>
inline void PIDController<T, U>::compute()
{   
   //Current time
   unsigned long currentTime = millis();
   float time_diff = (float)(currentTime - lastTime);
   if(time_diff >= sampleTime){
         // Substituting variables
         T setpoint = *setPoint_;
         T measurement = *measuredVal_;
         // error signal
         auto error = setpoint - measurement;
         // Proportional term
         pTerm = kp*error;
         // Integral term
         if (!(output_max > 0 && *controllerOutput_ >= output_max) && !(output_min < 0 && *controllerOutput_ <= output_min)) {
            errorSum += error * time_diff;
         }
         iTerm = ki * errorSum;

          if(iTerm > output_max){
            iTerm = output_max;
            }else if(iTerm < output_min){
            iTerm = output_min;
          }
         // Derivative term
        if (controllerType == pid_controller_type::typePID) {
            auto errorDt = (measurement - measurementPrev) / time_diff;
            dTerm = kd * errorDt;
         } else {
            dTerm = 0; // Set derivative term to zero for PI controller
          }

         // Output
         U Output = (U)pTerm + (U)iTerm + (U)dTerm;

         // setting the value for controller output
         *controllerOutput_ = Output;
         if(*controllerOutput_ > output_max){
            *controllerOutput_ = output_max;
         }else if(*controllerOutput_< output_min){
            *controllerOutput_ = output_min;
         }

         //Buffer variables to remember past values
         measurementPrev = measurement;
         lastTime = currentTime;

   }

}

template <typename T, typename U>
void PIDController<T, U>::setControllerType(pid_controller_type type)
{
   this->controllerType = type;
}

template <typename T, typename U>
void PIDController<T, U>::setSampleTime(float time)
{
   this->sampleTime = time;
}

template <typename T, typename U>
bool PIDController<T, U>::setOutputLimits(U minVal, U maxVal)
{
   if(minVal >= maxVal){
         return false;
      
   }
   this->output_max = maxVal;
   this->output_min = minVal;

   return true;
}


template <typename T, typename U>
bool PIDController<T, U>::init()
{  
   
   if (measuredVal_ == nullptr || setPoint_ == nullptr || controllerOutput_ == nullptr) {
        return false;
    }
    measurementPrev = *measuredVal_;
    iTerm = *controllerOutput_;
    return true;
}
