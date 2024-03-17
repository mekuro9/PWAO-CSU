/*==============================================================
// Filename :       ErrorHandler.cpp
// Version :        v1.0
// License :
// Description :    cpp file for error handling methods implementation
// Author :         Mrinal Magar
==============================================================*/
#include "error_handler/ErrorHandler.hpp"

ErrorHandler::ErrorHandler(rclcpp::Node *node, ErrorType type, const std::string &message){
    node_ = node;
    type_ = type;
    message_ = message;
    errorMsg_ = message;
}

// for now it's just logging. Ideally based on the error you would take different actions
void ErrorHandler::logError()
{
    auto logger = node_->get_logger();

    switch(type_){
        case ErrorType::NO_SENSOR_MESSAGE:
            RCLCPP_FATAL(logger, "No message from sensor publisher node:'\n' Message: %s ", message_.c_str());
            errorMsg_ = "ERROR: NO SENSOR DATA:  " + message_;
            break;
        case ErrorType::INCORRECT_INPUT:
            RCLCPP_ERROR(logger, "Incorrect input to the system:'\n' Message: %s ", message_.c_str());
            errorMsg_ = "ERROR: INCORRECT INPUT: " + message_;
            break;
        case ErrorType::COMMUNICATION_FAILURE:
            RCLCPP_FATAL(logger, "Communication failure:'\n' Message: %s ", message_.c_str());
            errorMsg_ = "ERROR: COMMUNICATION NOT ESTABLISHED " + message_;
            break;
    }
}

std::string ErrorHandler::getMessage(){
    return errorMsg_;
}