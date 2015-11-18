#include <ros/ros.h>

#include <ros/console.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>

#include <triangulator.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <stdlib.h>
#include <sstream>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>

#include <cv_bridge/cv_bridge.h>

using namespace cv;
using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

static ros::Publisher pose_pub;

Point2f findMarker(const sensor_msgs::ImageConstPtr& msg, const string& image_view);

const camera_params_t camera_params = {
    .resolution_h = 640,
    .resolution_v = 480,
    .fov_h = 80,
    .fov_v = 60,
    .D = 0.5
};

Point2f findMarker(const sensor_msgs::ImageConstPtr& msg, const string& image_view)
{
    Point2f pt;
	try
	{
		cv::Mat image = cv_bridge::toCvShare(msg, "mono8")->image;
        cv::Mat invert_image;
        bitwise_not(image, invert_image);

		// Setup SimpleBlobDetector parameters.
		cv::SimpleBlobDetector::Params params;
		
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
		imshow(image_view, invert_image);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
	}
    return pt;
}


        
void imageCallback(const ImageConstPtr& msg_left, 
        const ImageConstPtr& msg_right)
{
    static Point2f pt_left, pt_right;

    pt_left = findMarker(msg_left, "view_left");
    pt_right = findMarker(msg_right, "view_right");

    static Triangulator t(camera_params);

    if (pt_left.x != 0.0f && pt_right.x != 0.0f ) {
        geometry_msgs::Pose pose;
        geometry_msgs::PoseStamped ps;
        //ROS_INFO_THROTTLE(1, "L: (%5.2f, %5.2f), R: (%5.2f, %5.2f)", pt_left.x, pt_left.y, last_right.x, last_right.y);
        
        Point3f pt = t.get3DCoordinates(pt_left, pt_right);
        pose.position.x = pt.x;
        pose.position.y = pt.y;
        pose.position.z = pt.z;
        ps.pose = pose;
        ps.header.stamp = msg_left->header.stamp;
        ps.header.frame_id = "world";
        pose_pub.publish(ps);
        ROS_INFO_THROTTLE(1, "PT: (%5.2f, %5.2f, %5.2f)", pt.x, pt.y, pt.z);
    }
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "marker_tracker");
	ros::NodeHandle nh;

	cv::namedWindow("view_left");
	cv::namedWindow("view_right");
	cv::startWindowThread();


    message_filters::Subscriber<Image> left_sub(nh, "image_left", 1);
    message_filters::Subscriber<Image> right_sub(nh, "image_right", 1);

    typedef message_filters::sync_policies::ApproximateTime<Image, Image> MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), left_sub, right_sub);


    sync.registerCallback(boost::bind(&imageCallback, _1, _2));

    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 100);

    while (nh.ok()) {
        ros::spinOnce();
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
        tf::Quaternion q;
        q.setRPY(0, 0, 0);

        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "map"));
    }

	cv::destroyWindow("view_left");
	cv::destroyWindow("view_right");

	return 0;
}
