/*==============================================================
// Filename :       Point2Image.cpp
// Version :        v1.0
// License :  
// Description :    3D point to pixel coordinates
// Author :         Mrinal Magar
==============================================================*/


#include "lidar_visualization/Point2Image.hpp"
#include <cmath>
// 3D point to 2D camera coordinates

void Point2Image::setFocus(float x, float y, float cx, float cy)
{
    fx_ = x;
    fy_ = y;
    cx_ = cx;
    cy_ = cy;
}

void Point2Image::setImageSize(int x, int y)
{
    imageHSize_ = x;
    imageVSize_ = y;
    if(cx_ == 0){
         cx_ = imageHSize_/2;
    }
    if(cy_ == 0){
         cy_ = imageVSize_/2;
    }
   
    
}

void Point2Image::setFOV(int x,int y)
{
    HFOV_ = x;
    VFOV_ = y;
}


std::pair<float,float> Point2Image::getFocalLength()
{
    return std::pair<float,float>(fx_,fy_);
}


std::pair<int, int> Point2Image::getImageSize()
{
    return std::pair<int, int>(imageHSize_,imageVSize_);
}

std::pair<int, int> Point2Image::getFOV()
{
    return std::pair<int, int>(HFOV_,VFOV_);
}

double Point2Image::getXCam()
{
    return xCam_;
}

double Point2Image::getYCam()
{
    return yCam_;
}


double Point2Image::getZCam()
{
    return zCam_;
}

void Point2Image::setTransformationMatrix(std::vector<float>& array)
{   
    transformationMatrix_.clear();
    for(unsigned i=0; i<transformationMatrix_.size(); i++){
        transformationMatrix_.push_back(array.at(i));
    }
}

void Point2Image::setXCam(double x)
{
    xCam_ = x;
}

void Point2Image::setYCam(double y)
{
    yCam_ = y;
}

void Point2Image::setZCam(double z)
{
    zCam_ = z;
}

void Point2Image::transform2CameraFrame(double x, double y, double z)
{   
    double xCam = x*transformationMatrix_[0] + y*transformationMatrix_[1] + z*transformationMatrix_[2] + transformationMatrix_[3];
    double yCam = x*transformationMatrix_[4] + y*transformationMatrix_[5] + z*transformationMatrix_[6] + transformationMatrix_[7];
    double zCam = x*transformationMatrix_[8] + y*transformationMatrix_[9] + z*transformationMatrix_[10] + transformationMatrix_[11];
    setXCam(xCam);
    setYCam(yCam);
    setZCam(zCam);
}


std::pair<int, int> Point2Image::transformTo2DImage()
{   
    if(std::abs(zCam_)< 1e-6){
        return std::pair<int,int>(-1,-1);
    }
    double x_normalized = xCam_/zCam_;
    double y_normalized = yCam_/zCam_;

    int pixel_x = static_cast<int>(fx_*x_normalized + cx_);
    int pixel_y = static_cast<int>(fy_*y_normalized + cy_);

    if(((pixel_x <= 0 || pixel_x > imageHSize_) ||
    (pixel_y < 0 || pixel_y >= imageHSize_))){
        return std::pair<int,int>(-1,-1);
    }

    return std::pair<int, int>(pixel_x,pixel_y);
}


