#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <image_transport/image_transport.h>
#include <sstream>


/**
 * Given sensor image, return the XY coordinates of the marker in camera sensor
 * frame (0 in upper left corner).
 *
 * Optionally, shows the image in OpenCV's image view unless image_view == ""
 *
 */

cv::Point2f find_marker(const sensor_msgs::ImageConstPtr& msg, const std::string& image_view);
