#pragma once

#include <opencv2/core.hpp>
using namespace cv;

typedef struct {
    float resolution_h;
    float resolution_v;
    float fov_h;
    float fov_v;
    float D; //distance between cameras
} camera_params_t;

class Triangulator {
public:    
    Triangulator(const camera_params_t & p);
    void setParameters(const camera_params_t & p);
    
    /**
     * Gets angles from coordinates.
     *
     */
    Point2f getPointAngle(const Point2f& pt);
   
    Point3f get3DCoordinates(const Point2f & pt_left, const Point2f & pt_right);

private:
    camera_params_t parameters;
};
