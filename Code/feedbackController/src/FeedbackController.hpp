/*==============================================================
// Filename :       feedbackController.hpp
// Authors :        Mrinal Magar 
// Version :        v1.0
// License :  
// Description :    Header Abstract base class for feedback controller      
==============================================================*/

#ifndef FeedbackController_hpp
#define FeedbackController_hpp

//#include "Arduino.h"

template <typename T, typename U> class FeedbackController {
    public:

    // Constructor
    FeedbackController(){}

    // Deconstructor
    ~FeedbackController() = default;

    virtual void setup() = 0;
    virtual void compute(T setPoint, T measuredVal) = 0; //setpoint is the reference value; and measuredVal is the current sensor reading(current state)

};






#endif // FeedbackController