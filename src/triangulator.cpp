#include <ros/console.h>
#include "triangulator.h"
#include <string.h>
#include <math.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace cv;

Triangulator::Triangulator(const Mat& k1, const Mat &k2, const Mat &d1, 
            const Mat &d2, const Mat &r, const Mat& t):
    K_1(k1),
    K_2(k2),
    D_1(d1),
    D_2(d2),
    R(r),
    T(t),
    P_1(3, 4, CV_64F, 0.0f),
    P_2(3, 4, CV_64F, 0.0f)
{
    // P_1 = [I | 0]
    hconcat(cv::Mat::eye(3, 3, CV_64F), cv::Mat(3, 1, CV_64F, 0.0f), P_1);

    // P_2 = [R | T]
    hconcat(R, T, P_2);
}

Point3f Triangulator::get3DCoordinates(const Point2f & pt_left, const Point2f & pt_right) {
    
    //beware, won't compile, needs correct data types for each function
    Mat p_und_1, p_und_2;
    std::vector<Point2f> left_points;
    left_points.push_back(pt_left);
    
    std::vector<Point2f> right_points;
    right_points.push_back(pt_right);


    undistortPoints(left_points, p_und_1, K_1, D_1);
    undistortPoints(right_points, p_und_2, K_2, D_2);

    /*
    std::cout << "p_und_1: " << p_und_1 << std::endl; 
    std::cout << "p_und_2: " << p_und_2 << std::endl; 
    
    std::cout << "P_1" << std::endl << P_1 << std::endl;
    std::cout << "P_2" << std::endl << P_2 << std::endl;
    */

    //triangulate 
    Mat pt_4d;
    triangulatePoints(P_1, P_2, p_und_1, p_und_2, pt_4d);

   

    Point3f pt;
    //divide by scale factor
    pt.x = pt_4d.at<float>(0) / pt_4d.at<float>(3);
    pt.y = pt_4d.at<float>(1) / pt_4d.at<float>(3);
    pt.z = pt_4d.at<float>(2) / pt_4d.at<float>(3);

    return pt;
}

