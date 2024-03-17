/*==============================================================
// Filename :       ErrorHandler.hpp
// Version :        v1.0
// License :  
// Description :    Header file for error handling definitions
// Author :         Mrinal Magar
==============================================================*/
#include <rclcpp/rclcpp.hpp>
#include <string>

#ifndef ERRORHANDLER_HPP
#define ERRORHANDLER_HPP

enum class ErrorType {
    // no sensor message is for disconnected sensor or 
    // a sensor data publisher node not working
    NO_SENSOR_MESSAGE,

    // this is for incorrect or out of range user input,
    // but can also be for other incorrect inputs   
    INCORRECT_INPUT,

    //If communication with micro-ros or ros2web fails
    COMMUNICATION_FAILURE
};

class ErrorHandler {

    public:
    // constructor for errorhandler
    ErrorHandler(rclcpp::Node* node, ErrorType name, const std::string& message);

    ~ErrorHandler() = default;

    void logError();

    std::string getMessage();

    private:

    rclcpp::Node* node_;
    ErrorType type_;
    std::string message_;
    std::string errorMsg_;

};






#endif // ERRORHANDLER_HPP