/*==============================================================
// Filename :       feedbackController.hpp
// Authors :        Mrinal Magar 
// Version :        v1.0
// License :  
// Description :    Header Abstract base class for feedback controller      
==============================================================*/

#ifndef FeedbackController_h
#define FeedbackController_h

#include "Arduino.h"

template <typename T, typename U> class FeedbackController {
    public:

    // Constructor

    // Deconstructor
    virtual ~FeedbackController() = default;

    virtual bool setup(){
        return true;
    };
    //setpoint is the reference value; and measuredVal is the current sensor reading(current state)

    virtual void compute(){

    };
};


#endif // FeedbackController