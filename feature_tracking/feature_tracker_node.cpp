/*
    Node interface 
*/

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include "feature_tracker.h"
#include "../utility/pointDefinition.h"

CTrackerParam track_param;
CFeatureTracker feat_tracker(track_param); 

ros::Publisher *imagePointsLastPubPointer;
// ros::Publisher *imageShowPubPointer;

void imgCallback(const sensor_msgs::Image::ConstPtr& imageData);

int main(int argc, char* argv[]) 
{
    ros::init(argc, argv, "feature_tracking");
    ros::NodeHandle nh;
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    // ros::Subscriber imgDataSub = nh.subscribe<sensor_msgs::Image>("/image/raw", 1, imgCallback); 
    ros::Subscriber imgDataSub = nh.subscribe<sensor_msgs::Image>("/camera/rgb/image_rect", 1, imgCallback); 

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
    if(feat_tracker.handleImage(img_mono, img_time))
    {
	// publish msg 
	pcl::PointCloud<ImagePoint>::Ptr imagePointsLast(new pcl::PointCloud<ImagePoint>());
	imagePointsLast->points.resize(feat_tracker.mvPreImagePts.size()); 
	for(int i=0; i<feat_tracker.mvPreImagePts.size(); i++)
	{
	    ImagePoint& pt = imagePointsLast->points[i]; 
	    CFeatureTracker::ImagePoint& pt1 = feat_tracker.mvPreImagePts[i]; 
	    pt.u = pt1.u;  pt.v = pt1.v; pt.ind = pt1.ind; 
	}
	
	sensor_msgs::PointCloud2 imagePointsLast2;
	pcl::toROSMsg(*imagePointsLast,  imagePointsLast2); 
	imagePointsLast2.header.stamp = ros::Time().fromSec(feat_tracker.mTimePre); 
	
	imagePointsLastPubPointer->publish(imagePointsLast2); 
	// cout <<"feature_track_node: msg at" <<std::fixed<<imagePointsLast2.header.stamp.toSec()<<" first and last pt: "<<imagePointsLast->points[0].u<<" "<<imagePointsLast->points[imagePointsLast->points.size()-1].u<<" "<<imagePointsLast->points[imagePointsLast->points.size()-1].v<<endl;


    }
}

