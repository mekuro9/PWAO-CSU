/*==============================================================
// Filename :       Polar2Cartesian.hpp
// Version :        v1.0
// License :  
// Description :    Converting 3D coordinate to 2D image coordinate
// Author :         Mrinal Magar
==============================================================*/

#ifndef POINT2IMAGE_HPP
#define POINT2IMAGE_HPP

#include <utility>

class Point2Image {
    public:

        Point2Image(){
            
            // default values for OAK D Lite camera
            imageHSize_ = 4208;
            imageVSize_ = 3120;
            HFOV_ = 69;
            VFOV_ = 54;
            DFOV_ = 81;
            fx_ = 999;
            fy_ = 634;
            cx_ = 999;
            cy_ = 354;

        }

        ~Point2Image() = default;

        /// @brief 
        void setFocus();

        /// @brief 
        void setImageSize();

        /// @brief 
        void setFOV();

        /// @brief 
        void setCameraParam();

        /// @brief 
        void getCameraParam();

        /// @brief 
        /// @return 
        int getFocalLenght(); //TODO change this from into to std::pair

        /// @brief 
        /// @return 
        std::pair<int,int> getImageSize();

        /// @brief currently return only HFOV and VFOV
        /// @return 
        std::pair<int,int> getFOV();


        /// @brief 
        /// @param x 
        /// @param y 
        /// @param z 
        /// @return 
        std::pair<int,int> transformTo2DImage(double x, double y, double z);

    private:

    // Camera parameters
    int imageHSize_; //  horizontal image size in pixels
    int imageVSize_; // vertical image size
    int HFOV_; // horizontal field of view in degrees
    int VFOV_; // vertical field of view in degrees
    int DFOV_; // DFOV ?? unsure what this is
    double fx_,fy_,cx_,cy_; // Camera's intrinsic matrix
    

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

/color/camera_info topic

[fx 0   cx]
[0  fy  cy]
[0  0    1]

*/