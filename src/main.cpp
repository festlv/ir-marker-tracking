#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <stdlib.h>
#include <sstream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

bool findLed(cv::Mat& image, cv::Point2f& point) {

    cv::Scalar min(200);
    cv::Scalar max(255, 255, 255);
    cv::Mat masked_image;
    cv::inRange(image, min, max, masked_image);

    cv::SimpleBlobDetector::Params params;
    params.minDistBetweenBlobs = 50.0f;
    params.filterByInertia = false;
    params.filterByConvexity = false;
    params.filterByColor = false;
    params.filterByCircularity = false;
    params.filterByArea = true;
    params.minArea = 10.0f;
    params.maxArea = 100.0f;

    // set up and create the detector using the parameters
    cv::SimpleBlobDetector blob_detector(params);
    
    // detect!
    vector<cv::KeyPoint> detectedKeypoints;
    blob_detector.detect(masked_image, detectedKeypoints);
	
    if (detectedKeypoints.size() > 0) {
        //for now, let's assume the first one is correct
        cv::circle(image, detectedKeypoints[0].pt, 10, cv::Scalar(255));

        point.x = detectedKeypoints[0].pt.x;
        point.y = detectedKeypoints[0].pt.y;
        return true;
    }
    return false;
}

void imageCallback(const ImageConstPtr& msg_left, 
        const ImageConstPtr& msg_right)
{
    static cv::Point2f left_point, right_point;

	try
	{
		cv::Mat image_left = cv_bridge::toCvShare(msg_left, "mono8")->image;
		cv::Mat image_right = cv_bridge::toCvShare(msg_right, "mono8")->image;

        if (findLed(image_left, left_point) && findLed(image_right, right_point)) {
            ROS_INFO_THROTTLE(1, "Found point in both images\n");
            imshow("left", image_left);
            imshow("right", image_right);
        }

	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg_left->encoding.c_str());
	}
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "marker_tracker");
	ros::NodeHandle nh;

	cv::namedWindow("left");
	cv::namedWindow("right");

	cv::startWindowThread();

    message_filters::Subscriber<Image> left_sub(nh, "image_left", 1);
    message_filters::Subscriber<Image> right_sub(nh, "image_right", 1);

    typedef message_filters::sync_policies::ApproximateTime<Image, Image> MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), left_sub, right_sub);


    sync.registerCallback(boost::bind(&imageCallback, _1, _2));
     

	ros::spin();
	cv::destroyWindow("left");
	cv::destroyWindow("right");

	return 0;
}
