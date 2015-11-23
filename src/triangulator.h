#pragma once

#include <opencv2/core.hpp>

using cv::Mat;
using cv::Point2f;
using cv::Point3f;


class Triangulator {
public:    
    
    Triangulator(const Mat& k1, const Mat &k2, const Mat &d1, 
            const Mat &d2, const Mat &r, const Mat& t);

    Point3f get3DCoordinates(const Point2f & pt_left, const Point2f & pt_right);

protected:
    //intrinsic camera matrix (3x3)
    const Mat & K_1;
    const Mat & K_2;
   
    //disortion coefficients (1x5)
    const Mat & D_1;
    const Mat & D_2;
    
    //rotation and translation with respect to first camera 
    const Mat & R;
    const Mat & T;

    Mat P_1;
    Mat P_2; 
};
