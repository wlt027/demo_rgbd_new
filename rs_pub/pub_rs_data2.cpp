/*
    Oct. 8 2018, He Zhang, hzhang8@vcu.edu 
    
    get synchronized data from rs driver and publish it, not depend on message_filter 

*/


#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <typeinfo>

#include <chrono>
#include <algorithm>
#include <sys/stat.h>
#include <ros/ros.h>

#include <sensor_msgs/image_encodings.h>  
#include <std_msgs/String.h>  

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <queue>

#include <mutex>
#include <thread>
#include <condition_variable>
#include <string>

using namespace sensor_msgs;
using namespace std;
using namespace cv ; 

ros::Publisher rgb_pub; 
ros::Publisher dpt_pub; 

void publishRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgAD);

void rgbCb(const sensor_msgs::ImageConstPtr& msgRGB);
void dptCb(const sensor_msgs::ImageConstPtr& msgDpt);

queue<sensor_msgs::ImageConstPtr> rgb_buf; 
queue<sensor_msgs::ImageConstPtr> dpt_buf;
std::mutex m_buf; 
std::condition_variable con; 
vector<pair<sensor_msgs::ImageConstPtr, sensor_msgs::ImageConstPtr> > 
getMeasurements();

void process(); 

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "rs_publisher"); 
  ros::start(); 
  ros::NodeHandle nh; 

  ros::NodeHandle np("~"); 
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

  int q = 7; 

  rgb_pub = nh.advertise<sensor_msgs::Image>("/cam0/color", q); 
  dpt_pub = nh.advertise<sensor_msgs::Image>("/cam0/depth", q);

  ros::Subscriber rgb_sub2 = nh.subscribe("/camera/color/image_raw", q, rgbCb); 
  ros::Subscriber dpt_sub2 = nh.subscribe("/camera/aligned_depth_to_color/image_raw", q, dptCb); 
 
  ROS_WARN("realsense_publisher.cpp: start to subscribe msgs!"); 

  std::thread rs_publisher{process}; 
  ros::spin(); 
  ros::shutdown(); 

  return 0;
}


void process()
{
    while(ros::ok())
    {
    	vector<pair<sensor_msgs::ImageConstPtr, sensor_msgs::ImageConstPtr> > m; 
    	std::unique_lock<std::mutex> lk(m_buf); 
	con.wait(lk, [&]{
		return (m = getMeasurements()).size() > 0; 
	});
	lk.unlock(); 
	for(int i=0; i<m.size(); i++)
	{
	    sensor_msgs::ImageConstPtr rgb_ptr = m[i].first; 
	    sensor_msgs::ImageConstPtr dpt_ptr = m[i].second; 
	    publishRGBD(rgb_ptr, dpt_ptr); 	
	}
    }
}

vector<pair<sensor_msgs::ImageConstPtr, sensor_msgs::ImageConstPtr> > 
getMeasurements()
{
     vector<pair<sensor_msgs::ImageConstPtr, sensor_msgs::ImageConstPtr> > m; 

     while(true)
     {
     	if(rgb_buf.empty() || dpt_buf.empty()) 
	    return m; 
        while(!rgb_buf.empty() && rgb_buf.front()->header.stamp < dpt_buf.front()->header.stamp)
		rgb_buf.pop(); 
	if(rgb_buf.empty()) return m; 
	if(rgb_buf.front()->header.stamp == dpt_buf.front()->header.stamp)
	{
	   sensor_msgs::ImageConstPtr rgb_msg = rgb_buf.front(); 
	   rgb_buf.pop(); 
	   sensor_msgs::ImageConstPtr dpt_msg = dpt_buf.front(); 
	   dpt_buf.pop(); 
	   m.emplace_back(rgb_msg, dpt_msg); 
	}else if(rgb_buf.front()->header.stamp > dpt_buf.front()->header.stamp)
	{
		// ROS_ERROR("pub_rs_data_2: impossible rgb time > dpt time, failed to synchronize!");
		// return m;
		dpt_buf.pop(); 
	}
     }	
     return m ; 
}

void rgbCb(const sensor_msgs::ImageConstPtr& msgRGB)
{  
    ROS_DEBUG("receive rgb msg at time: %f", msgRGB->header.stamp.toSec()); 
    m_buf.lock(); 
    	rgb_buf.push(msgRGB); 
    m_buf.unlock(); 
    con.notify_one(); 
    return ; 
}
void dptCb(const sensor_msgs::ImageConstPtr& msgAD)
{
    ROS_INFO("receive dpt msg at time: %f", msgAD->header.stamp.toSec()); 
    m_buf.lock(); 
    	dpt_buf.push(msgAD); 
    m_buf.unlock(); 
    con.notify_one(); 
    return; 
}


void publishRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgAD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    cv_bridge::CvImageConstPtr cv_ptrAD;

    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB, sensor_msgs::image_encodings::BGR8);
        cv_ptrAD = cv_bridge::toCvShare(msgAD, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    stringstream ss;
    ss << msgRGB->header.stamp; 
    ROS_DEBUG("pub_rs_data2: publish rgbd msg: %s" , ss.str().c_str()); 

    rgb_pub.publish(*(cv_ptrRGB.get())); 
    dpt_pub.publish(*(cv_ptrAD.get())); 
    ros::spinOnce(); 

    return; 
}




