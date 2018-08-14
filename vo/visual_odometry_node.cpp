/*
    Aug. 14 2018, He Zhang, hzhang8@vcu.edu 

    visual odometry node
*/

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "visual_odometry.h"

VisualOdometry vo; 

ros::Publisher *voDataPubPointer = NULL;
tf::TransformBroadcaster *tfBroadcasterPointer = NULL;
ros::Publisher *depthPointsPubPointer = NULL;
ros::Publisher *imagePointsProjPubPointer = NULL;
ros::Publisher *imageShowPubPointer;

void imuDataHandler(const sensor_msgs::Imu::ConstPtr& imuData)
{
    //TODO
}
void depthCloudHandler(const sensor_msgs::PointCloud2ConstPtr& depthCloud2)
{
    vo.depthCloudHandler(depthCloud2); 
    return ; 
}

void imagePointsHandler(const sensor_msgs::PointCloud2ConstPtr& imagePoints2)
{
    vo.imagePointsHandler(imagePoints2); 
    // publish msg voData 
    nav_msgs::Odometry voData; 
    voData.header.frame_id = "/camera_init"; 
    voData.header.stamp = imagePoints2->header.stamp;
    voData.child_frame_id = "/camera";
 
    tf::Quaternion q = vo.mCurrPose.getRotation(); 
    tf::Vector3 t = vo.mCurrPose.getOrigin(); 
    voData.pose.pose.orientation.x = q.getX(); 
    voData.pose.pose.orientation.y = q.getY(); 
    voData.pose.pose.orientation.z = q.getZ(); 
    voData.pose.pose.orientation.w = q.getW(); 
    voData.pose.pose.position.x = t.getX(); 
    voData.pose.pose.position.y = t.getY();
    voData.pose.pose.position.z = t.getZ(); 
    voDataPubPointer->publish(voData);
    
    // broadcast voTrans 
    tf::StampedTransform voTrans;
    voTrans.frame_id_ = "/camera_init";
    voTrans.child_frame_id_ = "/camera";
    voTrans.stamp_ = imagePoints2->header.stamp;
    voTrans.setRotation(q); 
    voTrans.setOrigin(t); 
    tfBroadcasterPointer->sendTransform(voTrans); 
    
    // TODO: handle msg depth point

}
void imageDataHandler(const sensor_msgs::Image::ConstPtr& imageData);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visualOdometry");
  ros::NodeHandle nh;

  ros::Subscriber imagePointsSub = nh.subscribe<sensor_msgs::PointCloud2>
                                   ("/image_points_last", 5, imagePointsHandler);

  ros::Subscriber depthCloudSub = nh.subscribe<sensor_msgs::PointCloud2> 
                                  ("/depth_cloud", 5, depthCloudHandler);

  // ros::Subscriber imuDataSub = nh.subscribe<sensor_msgs::Imu> ("/imu/data", 5, imuDataHandler);

  ros::Publisher voDataPub = nh.advertise<nav_msgs::Odometry> ("/cam_to_init", 5);
  voDataPubPointer = &voDataPub;

  tf::TransformBroadcaster tfBroadcaster;
  tfBroadcasterPointer = &tfBroadcaster;

  // ros::Publisher depthPointsPub = nh.advertise<sensor_msgs::PointCloud2> ("/depth_points_last", 5);
  // depthPointsPubPointer = &depthPointsPub;

  // ros::Publisher imagePointsProjPub = nh.advertise<sensor_msgs::PointCloud2> ("/image_points_proj", 1);
  // imagePointsProjPubPointer = &imagePointsProjPub;

  // ros::Subscriber imageDataSub = nh.subscribe<sensor_msgs::Image>("/image/show", 1, imageDataHandler);

  // ros::Publisher imageShowPub = nh.advertise<sensor_msgs::Image>("/image/show_2", 1);
  // imageShowPubPointer = &imageShowPub;

  ros::spin();

  return 0;
}


void imageDataHandler(const sensor_msgs::Image::ConstPtr& imageData) 
{
//
//  cv_bridge::CvImagePtr bridge = cv_bridge::toCvCopy(imageData, "bgr8");
//
//  int ipRelationsNum = ipRelations->points.size();
//  for (int i = 0; i < ipRelationsNum; i++) {
//    if (fabs(ipRelations->points[i].v) < 0.5) {
//      cv::circle(bridge->image, cv::Point((kImage[2] - ipRelations->points[i].z * kImage[0]) / showDSRate,
//                (kImage[5] - ipRelations->points[i].h * kImage[4]) / showDSRate), 1, CV_RGB(255, 0, 0), 2);
//    } else if (fabs(ipRelations->points[i].v - 1) < 0.5) {
//      cv::circle(bridge->image, cv::Point((kImage[2] - ipRelations->points[i].z * kImage[0]) / showDSRate,
//                (kImage[5] - ipRelations->points[i].h * kImage[4]) / showDSRate), 1, CV_RGB(0, 255, 0), 2);
//    } else if (fabs(ipRelations->points[i].v - 2) < 0.5) {
//      cv::circle(bridge->image, cv::Point((kImage[2] - ipRelations->points[i].z * kImage[0]) / showDSRate,
//                (kImage[5] - ipRelations->points[i].h * kImage[4]) / showDSRate), 1, CV_RGB(0, 0, 255), 2);
//    } /*else {
//      cv::circle(bridge->image, cv::Point((kImage[2] - ipRelations->points[i].z * kImage[0]) / showDSRate,
//                (kImage[5] - ipRelations->points[i].h * kImage[4]) / showDSRate), 1, CV_RGB(0, 0, 0), 2);
//    }*/
//  }
//
//  sensor_msgs::Image::Ptr imagePointer = bridge->toImageMsg();
//  imageShowPubPointer->publish(imagePointer);
}



