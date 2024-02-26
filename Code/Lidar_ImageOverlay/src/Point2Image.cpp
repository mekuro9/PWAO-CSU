/*==============================================================
// Filename :       Polar2Cartesian.hpp
// Version :        v1.0
// License :  
// Description :    Converting polar coordiates to cartesian coordinates
// Author :         Mrinal Magar
==============================================================*/


#include "Point2Image.hpp"
// 3D point to 2D camera coordinates

void Point2Image::setFocus()
{
    //TODO
}

void Point2Image::setImageSize()
{
    //TODO
}

void Point2Image::setFOV()
{
    //TODO
}

void Point2Image::getCameraParam()
{
    //TODO
}

void Point2Image::setCameraParam()
{
    //TODO
}

int Point2Image::getFocalLenght()
{
    return 0;
}


std::pair<int, int> Point2Image::getImageSize()
{
    return std::pair<int, int>(imageHSize_,imageVSize_);
}

std::pair<int, int> Point2Image::getFOV()
{
    return std::pair<int, int>(HFOV_,VFOV_);
}

std::pair<int, int> Point2Image::transformTo2DImage(double x, double y, double z)
{   
    if(std::abs(z)< 1e-6){
        return std::pair<int,int>(-1,-1);
    }
    double x_normalized = x/z;
    double y_normalized = y/z;

    int pixel_x = static_cast<int>(fx_*x_normalized + cx_);
    int pixel_y = static_cast<int>(fy_*y_normalized + cy_);

    return std::pair<int, int>(pixel_x,pixel_y);
}
