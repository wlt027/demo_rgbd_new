/*
    Node interface 
*/


#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include "feature_tracker.h"

CTrackerParam track_param;
CFeatureTracker feat_tracker(track_param); 

ros::Publisher *imagePointsLastPubPointer;
// ros::Publisher *imageShowPubPointer;

void imgCallback(const sensor_msgs::Image::ConstPtr& imageData);

int main(int argc, char* argv[]) 
{
    ros::init(argc, argv, "featureTracking");
    ros::NodeHandle nh;
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    ros::Subscriber imgDataSub = nh.subscribe<sensor_msgs::Image>("/image/raw", 1, imgCallback); 

   
    ros::Publisher tracked_features_pub = nh.advertise<sensor_msgs::PointCloud2>("/image_points_last", 5); 
    imagePointsLastPubPointer = &tracked_features_pub;

    ros::spin();

    return 1; 
}


void imgCallback(const sensor_msgs::Image::ConstPtr& _img)
{
    cv_bridge::CvImageConstPtr ptr = cv_bridge::toCvCopy(_img, sensor_msgs::image_encodings::MONO8);

    cv::Mat img_mono = ptr->image; 
    double img_time = _img->header.stamp.toSec(); 
    feat_tracker.handleImage(img_mono, img_time); 
}

