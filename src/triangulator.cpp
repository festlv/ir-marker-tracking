#include <ros/console.h>
#include "triangulator.h"
#include <string.h>
#include <math.h>

#define DEG_TO_RAD(x) (0.0174532925f * x)

Triangulator::Triangulator(const camera_params_t & p) {
    setParameters(p);
}

void Triangulator::setParameters(const camera_params_t & p) {
    parameters = p;
}

Point2f Triangulator::getPointAngle(const Point2f& pt) {
    /* phi = (180 - FOV_h) / 2 + (FOV_h/RES_h) * (RES_h - I_x) */

    Point2f angle;
    angle.x = (180.0f - parameters.fov_h) / 2 + \
              (parameters.fov_h / parameters.resolution_h) * (parameters.resolution_h - pt.x);

    angle.y = (180.0f - parameters.fov_v) / 2 + \
              (parameters.fov_v / parameters.resolution_v) * (parameters.resolution_v - pt.y);
    return angle; 
}


Point3f Triangulator::get3DCoordinates(const Point2f & pt_left, const Point2f & pt_right) {
    Point2f phi_left = getPointAngle(pt_left);
    Point2f phi_right = getPointAngle(pt_right);

    phi_right.x = 180 - phi_right.x;

    ROS_INFO_THROTTLE(1, "Phi_left_x: %6.2f, Phi_right_x: %6.2f", (float)phi_left.x, (float)phi_right.x); 

    Point3f pt;

    pt.y = parameters.D / (1.0f / tanf(DEG_TO_RAD(phi_left.x)) + 1.0f / tanf(DEG_TO_RAD(phi_right.x)));

    pt.x = parameters.D - (tanf(DEG_TO_RAD(phi_left.x)) * parameters.D) / (tanf(DEG_TO_RAD(phi_left.x)) + tanf(DEG_TO_RAD(phi_right.x)));
    
    pt.z = pt.y / (sinf(DEG_TO_RAD(phi_left.y)));

    return pt;
}

