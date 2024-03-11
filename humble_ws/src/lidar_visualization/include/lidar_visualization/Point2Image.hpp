/*==============================================================
// Filename :       Point2Image.hpp
// Version :        v1.0
// License :  
// Description :    Converting 3D coordinate to 2D image  pixel coordinate
// Author :         Mrinal Magar
==============================================================*/

#ifndef POINT2IMAGE_HPP
#define POINT2IMAGE_HPP

#include <utility>
#include <vector>

class Point2Image {
    public:

        Point2Image(){
            
            // default values for OAK D Lite camera
            imageHSize_ = 1280;
            imageVSize_ = 720;
            HFOV_ = 1.2;//69; // degrees
            VFOV_ = 0.94;//54;
            DFOV_ = 81;
            fx_ = 999.89575;
            fy_ = 999.89575;
            cx_ = 634.74695;
            cy_ = 354.51739;
            transformationMatrix_ = {1.0, 0.0, -0.54402111,0.0544021111,0.0,1.0,0.0,0.0, 0.54402111,0.0,-0.83907153, 0.0839071529,0.0,0.0,0.0,1.0};

        }

        ~Point2Image() = default;

        /// @brief 
        void setFocus(float, float, float, float);

        /// @brief 
        void setImageSize(int, int);

        /// @brief 
        void setFOV(int, int);

        /// @brief 
        /// @return 
        std::pair<float,float> getFocalLength();

        /// @brief 
        /// @return 
        std::pair<int,int> getImageSize();

        /// @brief currently return only HFOV and VFOV
        /// @return 
        std::pair<int,int> getFOV();
        double getXCam();
        double getYCam();
        double getZCam();

        /// @brief 
        void setTransformationMatrix(std::vector<float>& array);

         /// @brief 
        void transform2CameraFrame(double, double, double);
        
        /// @brief 
        /// @param x 
        /// @param y 
        /// @param z 
        /// @return 
        std::pair<int,int> transformTo2DImage();

        /// @brief 
        /// @param  
        void setXCam(double);
        /// @brief 
        /// @param  
        void setYCam(double);
        /// @brief 
        /// @param  
        void setZCam(double);

    private:

    // Camera parameters
    int imageHSize_; //  horizontal image size in pixels
    int imageVSize_; // vertical image size
    int HFOV_; // horizontal field of view in degrees
    int VFOV_; // vertical field of view in degrees
    int DFOV_; // DFOV ?? unsure what this is
    double fx_,fy_,cx_,cy_; // Camera's intrinsic matrix
    
    //transformation matrix is 4x4
    std::vector<float> transformationMatrix_ ; // transformation matrix depends on the geometry
    // Coordinates in CameraFrame
    double xCam_, yCam_, zCam_;
    
};

#endif // Point2Image_hpp

/*
k:
- 999.895751953125
- 0.0
- 634.7469482421875
- 0.0
- 999.895751953125
- 354.51739501953125
- 0.0
- 0.0
- 1.0

but use P 4x4 matrix
camerainfo P matrix is float[12]
/color/camera_info topic

[fx 0   cx]
[0  fy  cy]
[0  0    1]

*/