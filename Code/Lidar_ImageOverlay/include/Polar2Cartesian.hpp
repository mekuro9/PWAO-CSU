
/*==============================================================
// Filename :       Polar2Cartesian.hpp
// Version :        v1.0
// License :  
// Description :    Converting polar coordiates to cartesian coordinates
// Author :         Mrinal Magar
==============================================================*/

#ifndef POLAR2CARTESIAN_HPP
#define POLAR2CARTESIAN_HPP

#include <cmath>
#include <utility>

class Polar2Cartesian {
    public:

        Polar2Cartesian(){
            angle_ = 0;
            radius_ = 0;
            x_ = 0;
            y_ = 0;
        }

        ~Polar2Cartesian() = default;

        /// @brief 
        /// @param r 
        /// @param theta 
        void setPolar(const double r, const double theta){ 
            radius_ = r;
            angle_ = theta;
        }

        /// @brief 
        void polar2Cartesian(){

            x_ = radius_*std::cos(angle_);
            y_ = radius_*std::sin(angle_); // angle in radians

        }

        /// @brief 
        /// @return 
        double getX() const{
            return x_;
        }

        /// @brief 
        /// @return 
        double getY() const{
            return y_;
        }

        /// @brief 
        /// @param r 
        /// @param theta 
        /// @return 
        std::pair<double, double> Convert2Cartesian(const double r, const double theta){

            setPolar(r,theta);
            polar2Cartesian();

            return {getX(),getY()};
        }

        
    private:

    // Angle in radians
    double angle_;
    double radius_; // radius is the range of laserScan
    double x_, y_;
};

#endif // Polar2Cartesian_hpp