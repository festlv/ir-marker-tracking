#include "marker_finder.h"

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include "misc.h"

using namespace cv;
using namespace std;

Point2f find_marker(const sensor_msgs::ImageConstPtr& msg, const string& image_view)
{
    Point2f pt;
    static cv::Mat image;
    static cv::Mat invert_image;

	try
	{
        image = cv_bridge::toCvShare(msg, "mono8")->image;

        //blob detector only works for dark blobs
        bitwise_not(image, invert_image);

		// Setup SimpleBlobDetector parameters.
		static cv::SimpleBlobDetector::Params params;
		
		// Change thresholds
		params.minThreshold = 0;
		params.maxThreshold = 150;
		
		// Filter by Area.
		//params.filterByArea = true;
		params.minArea = 4;
		params.maxArea = 500;
		
		// Set up detector with params
		cv::SimpleBlobDetector detector(params);
		
		std::vector<KeyPoint> keypoints;
		detector.detect(invert_image, keypoints);

        if (keypoints.size() > 0) {
            for (int i = 0; i < keypoints.size(); i++) {
                circle(invert_image, keypoints[i].pt, 10, Scalar(0,0,255), 2);
            }
            pt.x = keypoints[0].pt.x;
            pt.y = keypoints[0].pt.y;

        }
        if (image_view != "") {
            imshow(image_view, invert_image);
        }
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
	}
    return pt;
}
