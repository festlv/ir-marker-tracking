#include <ros/ros.h>

#include <ros/topic.h>

#include <ros/console.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_geometry/pinhole_camera_model.h>

#include <tf/transform_broadcaster.h>

#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <stdlib.h>
#include <sstream>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "triangulator.h"
#include "marker_finder.h"


using namespace cv;
using namespace std;
using namespace sensor_msgs;
using namespace message_filters;
using namespace image_geometry;

static ros::Publisher pose_pub;

static Triangulator *t;

void imageCallback(const ImageConstPtr& msg_left, 
        const ImageConstPtr& msg_right)
{
    static Point2f pt_left, pt_right;

    pt_left = find_marker(msg_left, "view_left");
    pt_right = find_marker(msg_right, "view_right");

    if (pt_left.x != 0.0f && pt_right.x != 0.0f ) {
        geometry_msgs::Pose pose;
        geometry_msgs::PoseStamped ps;
        ROS_INFO_THROTTLE(1, "L: (%5.2f, %5.2f), R: (%5.2f, %5.2f)", pt_left.x, pt_left.y, pt_right.x, pt_right.y);
         
        Point3f pt = t->get3DCoordinates(pt_left, pt_right);
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

    sensor_msgs::CameraInfoConstPtr left_camera_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("camera_info_left", nh, ros::Duration(5.0f));
    sensor_msgs::CameraInfoConstPtr right_camera_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("camera_info_right", nh, ros::Duration(5.0f));
    
    if (!left_camera_info || !right_camera_info) {
        std::cerr << "No camera_info messages received, exiting!";
        return 0;
    }


    cv::Mat R_mat = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat T_mat = (cv::Mat_<double>(3, 1) << -0.5f, 0, 0);
    
    cv::Mat K_l = cv::Mat(3, 3, CV_64F, (void*)left_camera_info->K.data());
    cv::Mat K_r = cv::Mat(3, 3, CV_64F, (void*)right_camera_info->K.data());

    cv::Mat D_l = cv::Mat(1, 5, CV_64F, (void*)left_camera_info->D.data());
    cv::Mat D_r = cv::Mat(1, 5, CV_64F, (void*)right_camera_info->D.data());
   
    /*
    std::cout << "K_l\n" << K_l << std::endl;
    std::cout << "D_l\n" << D_l << std::endl;
    std::cout << "R\n" << R_mat << std::endl;
    std::cout << "T\n" << T_mat << std::endl;
    */ 

    t = new Triangulator(K_l, K_r, D_l, D_r, R_mat, T_mat);

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
    
    delete t;
	cv::destroyWindow("view_left");
	cv::destroyWindow("view_right");

	return 0;
}
