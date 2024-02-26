/*==============================================================
// Filename :       Polar2Cartesian.hpp
// Version :        v1.0
// License :  
// Description :    Overlaying 3D point on to camera image
// Author :         Mrinal Magar
==============================================================*/

#ifndef PROJECTION2IMAGE_HPP
#define PROJECTION2IMAGE_HPP

#include <utility>
#include <vector>

template <typename T>
class Projection2Image {
    public:

        Projection2Image(){

        }

        ~Projection2Image() = default;

        /// @brief 
        void setTransformationMatrix(T);

        /// @brief 
        void transform2CameraFrame();

        /// @brief 
        void project2Camera();



    private:
        //camerainfo P matrix is float[12]
        std::vector<float> transformationMatrix_;

};

#endif


