/*
    Aug. 12 2018, He Zhang, hzhang8@vcu.edu 

    A depth handler node
*/

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include "../vo/depth_handler.h"
using namespace std; 

ros::Publisher *depthCloudPubPointer = NULL;
ros::Publisher *currDepthCloudPubPointer = NULL;

DepthHandler* pDptHandler = new DepthHandler(); 

void voDataHandler(const nav_msgs::Odometry::ConstPtr& voData); 
void syncCloudHandler(const sensor_msgs::Image::ConstPtr& syncCloud2); 

int main(int argc, char** argv)
{
  ros::init(argc, argv, "depth_handler");
  ros::NodeHandle nh;
  
  ros::Subscriber voDataSub = nh.subscribe<nav_msgs::Odometry> ("/cam_to_init", 5, voDataHandler);
  ros::Subscriber syncCloudSub = nh.subscribe<sensor_msgs::Image>
	("/cam0/depth", 5, syncCloudHandler);

  ros::Publisher depthCloudPub = nh.advertise<sensor_msgs::PointCloud2> ("/depth_cloud", 5);
  depthCloudPubPointer = &depthCloudPub;

  ros::Publisher cur_depthCloudPub = nh.advertise<sensor_msgs::PointCloud2> ("/current_depth_cloud", 5);
  currDepthCloudPubPointer = &cur_depthCloudPub;

  ros::spin();

  return 0;
}


void syncCloudHandler(const sensor_msgs::Image::ConstPtr& syncCloud2)
{
    pDptHandler->cloudHandler2(syncCloud2); 
    sensor_msgs::PointCloud2 depthCloud2; 
    pcl::toROSMsg(*(pDptHandler->mCloudArray[pDptHandler->mSyncCloudId]), depthCloud2);
    depthCloud2.header.frame_id = "camera"; 
    depthCloud2.header.stamp = syncCloud2->header.stamp; 
    currDepthCloudPubPointer->publish(depthCloud2);

    return; 
}

void voDataHandler(const nav_msgs::Odometry::ConstPtr& voData)
{
    pDptHandler->voDataHandler(voData);
    
    // publish the result 
    sensor_msgs::PointCloud2 depthCloud2;
    pcl::toROSMsg(*(pDptHandler->mCloudPub), depthCloud2);
    depthCloud2.header.frame_id = "camera";
    depthCloud2.header.stamp = voData->header.stamp;
    depthCloudPubPointer->publish(depthCloud2);
    // cout<<"depth_handler.cpp: publish depth cloud "<<depthCloud2.height * depthCloud2.width<<" points"<<endl;
    return ; 
}



