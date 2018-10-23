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
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>  

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

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

bool b_publish_pc = false; 
ros::Publisher pc_pub; 
double fx, fy, cx, cy; 

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
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

  int q = 7; 

  np.param("publish_point_cloud", b_publish_pc, b_publish_pc); 
  if(b_publish_pc)
  {
  	pc_pub = nh.advertise<sensor_msgs::PointCloud2>("/depth_cloud", q); 
  	np.param("fx", fx, 617.31); 
	np.param("fy", fy, 617.71); 
	np.param("cx", cx, 326.24); 
	np.param("cy", cy, 239.97); 
	ROS_WARN("publish_pc: given fx = %f fy = %f cx = %f cy = %f", fx, fy, cx, cy); 
  }


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

void processCloud(const sensor_msgs::Image::ConstPtr& dpt_img_msg)
{   
     pcl::PointCloud<pcl::PointXYZ>::Ptr tmpPC(new pcl::PointCloud<pcl::PointXYZ>); 
     cv::Mat dpt_img = cv_bridge::toCvCopy(dpt_img_msg)->image; 
     cv::Mat dst; 
     cv::medianBlur(dpt_img, dst, 5); 
     dpt_img = dst; 

     float scale = 0.001; 
     float max_dis = 3.; 
     float min_dis = 0.3; 
     float ds = 5; 
     float half_ds =ds/2. - 0.5; 
     for(double i=half_ds; i < dpt_img.rows; i+= ds)
     for(double j=half_ds; j < dpt_img.cols; j+= ds)
     {
     	int pixCnt = 0; 
	float vd, vd_sum = 0; 
	int is = (int)(i-half_ds); int ie = (int)(i+half_ds); 
	int js = (int)(j-half_ds); int je = (int)(j+half_ds); 
	for(int ii=is; ii<=ie; ii++)
	for(int jj=js; jj<=je; jj++)
	{
	    unsigned short _dpt = dpt_img.at<unsigned short>(ii, jj); 
	    vd = _dpt * scale; 
	    if(vd <= max_dis && vd >= min_dis)
	    {
	    	vd_sum += vd; 
		++pixCnt; 
	    }
	}

	if(pixCnt > 0)
	{
	    double u = (j-cx)/fx; 
	    double v = (i-cy)/fy;
	    double mean_vd = vd_sum/pixCnt; 
	    pcl::PointXYZ pt; 
	    pt.x = u * mean_vd; 
	    pt.y = v * mean_vd; 
	    pt.z = mean_vd; 
	    tmpPC->points.push_back(pt);
	}
     }
     tmpPC->width = tmpPC->points.size(); 
     tmpPC->height = 1; 
     tmpPC->is_dense = true; 
     
     // publish it
     sensor_msgs::PointCloud2 depthCloud2; 
     pcl::toROSMsg(*tmpPC, depthCloud2); 
     depthCloud2.header.frame_id = "map"; 
     depthCloud2.header.stamp = dpt_img_msg->header.stamp; 
     pc_pub.publish(depthCloud2); 
     return ; 
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
	    if(b_publish_pc && i==m.size()-1)
	    {
	    	processCloud(dpt_ptr); 
	    }
	    ros::spinOnce(); 
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
    ROS_DEBUG("receive dpt msg at time: %f", msgAD->header.stamp.toSec()); 
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




