#include <ros/ros.h>

#include <ros/console.h>

#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <stdlib.h>
#include <sstream>
#include <math.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "marker_finder.h"

#include <boost/format.hpp>
#include "misc.h"

using namespace cv;
using namespace std;
using namespace sensor_msgs;
using namespace message_filters;


const float D = 0.5f;

const float N = 3.0f;

void imageCallback(const ImageConstPtr& msg_left, 
        const ImageConstPtr& msg_right)
{
    static Point2f pt_left, pt_right;

    pt_left = find_marker(msg_left, "");
    pt_right = find_marker(msg_right, "");
    
    static cv::Mat image_left;
    static cv::Mat image_right;


    image_left = cv_bridge::toCvShare(msg_left, "mono8")->image;
    image_right = cv_bridge::toCvShare(msg_right, "mono8")->image;
        
    cv::Size size = image_left.size();
    const float side_offset = 5;
    cv::Mat image_out(size.height, size.width * 2 + side_offset, image_left.type(), Scalar(255,255,255));

    image_left.copyTo(image_out(Rect(0, 0, image_left.cols, image_left.rows)));
    image_right.copyTo(image_out(Rect(image_right.cols + side_offset, 0, image_right.cols, image_right.rows)));


    static Point2f left_closest_point;
    static Point2f right_closest_point;

    //corresponding coordinates from the other image
    static Point2f left_closest_point_right;
    static Point2f right_closest_point_left;
    
    Point2f center;
    center.x = size.width / 2;

    //draw a line representing image center
    line(image_out, Point(center.x, 0), 
            Point(center.x, size.height), Scalar(255));

    line(image_out, Point(center.x + size.width + side_offset, 0), 
            Point(center.x + size.width + side_offset, size.height), 
            Scalar(255));


    if (pt_left.x != 0.0f && pt_right.x != 0.0f ) {
        //got a marker in both images, keep track of the marker which is
        //closest to center for each camera
        if (fabsf(center.x - pt_left.x) < fabsf(center.x - left_closest_point.x)) {
            //got a new closest point
            left_closest_point = pt_left;
            left_closest_point_right = pt_right;
        }

        if (fabsf(center.x - pt_right.x) < fabsf(center.x - right_closest_point.x)) {
            //got a new closest point
            right_closest_point = pt_right;
            right_closest_point_left = pt_left;
        }
        //draw circles for the best matches
        circle(image_out, left_closest_point, 5, Scalar(255));
        circle(image_out, 
                Point(right_closest_point.x + size.width + side_offset, 
                    right_closest_point.y), 5, Scalar(255));
       
    }

    //calculate error for both cameras
    float error_left = fabsf(center.x - left_closest_point.x);
    float error_right = fabsf(center.x - right_closest_point.x);

    char text_left[64];
    char text_right[64];

    snprintf(text_left, 64, "Error left: %.2f", error_left);
    snprintf(text_right, 64, "Error right: %.2f", error_right);

    putText(image_out, std::string(text_left), Point(center.x, size.height - 20), 
            FONT_HERSHEY_PLAIN, 1.0, Scalar(255));

    putText(image_out, std::string(text_right), Point(center.x + size.width + side_offset, 
                size.height - 20), 
            FONT_HERSHEY_PLAIN, 1.0, Scalar(255));
    
    if (error_left < 1.0f) {
        float alpha = RAD_TO_DEG(atanf(N / D));
        float fov = (90 - alpha) / (0.5 - left_closest_point_right.x / size.width);
        ROS_INFO_THROTTLE(1, "\n========\n");

        ROS_INFO_THROTTLE(1, "P_L: (%.2f, %.2f), P_R: (%.2f, %.2f)\n", left_closest_point.x, left_closest_point.y, left_closest_point_right.x, left_closest_point_right.y);
        ROS_INFO_THROTTLE(1, "FOV: %.2f\n", fov);

    }

    imshow("view", image_out);
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "marker_tracker");
	ros::NodeHandle nh;

	cv::namedWindow("view");
	cv::startWindowThread();


    message_filters::Subscriber<Image> left_sub(nh, "image_left", 1);
    message_filters::Subscriber<Image> right_sub(nh, "image_right", 1);

    typedef message_filters::sync_policies::ApproximateTime<Image, Image> MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), left_sub, right_sub);


    sync.registerCallback(boost::bind(&imageCallback, _1, _2));

    
    ROS_INFO("For calibration, place the marker at %.1fm distance from camera plane.\n", N);
    ROS_INFO("Place it as close as possible in the center of the left camera, and then the right camera\n");
    ROS_INFO("Distance between cameras: %.1fm\n", D);

    ros::spin();

	cv::destroyWindow("view");

	return 0;
}
