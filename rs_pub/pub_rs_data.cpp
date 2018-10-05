/*
    Oct. 5 2018, He Zhang, hzhang8@vcu.edu 
    
    get synchronized data from rs driver and publish it 

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

#include <opencv2/objdetect/objdetect.hpp>  
#include <opencv2/imgproc/imgproc.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/core/core.hpp>

#include <std_msgs/String.h>  

#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <mutex>
#include <string>

using namespace sensor_msgs;
using namespace message_filters;

using namespace std;
using namespace cv ; 

ofstream tsfile;

int frame_id = 0 ;  

string gDir("/home/jin/Data_Capture/myImage");
string gDataName("OBS");

ros::Publisher rgb_pub; 
ros::Publisher dpt_pub; 

void ImageGrabberRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgAD);
void ImageGrabberFull(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgAD, const sensor_msgs::ImageConstPtr& msgD, const sensor_msgs::ImageConstPtr& msgIr1, const sensor_msgs::ImageConstPtr& msgIr2);

void rgbCb(const sensor_msgs::ImageConstPtr& msgRGB);
void dptCb(const sensor_msgs::ImageConstPtr& msgDpt);

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "save_data"); 
  ros::start(); 
  ros::NodeHandle nh; 

  ros::NodeHandle np("~"); 
  np.param("data_dir", gDir, gDir); 
  np.param("data_name", gDataName, gDataName); 

  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

  string d_dir = gDir +"/"+ gDataName; 
  string d_rgb = d_dir + "/color"; 
  // string d_Adpt = d_dir + "/aligned_depth";
  string d_dpt = d_dir + "/depth"; 
  // string d_ir = d_dir + "/ir"; 
  // string d_ir2 = d_dir + "/ir2"; 

  // mkdir(d_dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  // mkdir(d_rgb.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  // mkdir(d_dpt.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  // mkdir(d_ir.c_str(),  S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  // mkdir(d_ir2.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

  // string f_time = d_dir + "/timestamp.txt"; 
  // tsfile.open(f_time.c_str());
  // tsfile << "index  timestamp\n";

  int q = 7; 

  rgb_pub = nh.advertise<sensor_msgs::Image>("/cam0/color", q); 
  dpt_pub = nh.advertise<sensor_msgs::Image>("/cam0/depth", q);

  message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/color/image_raw", q);
  message_filters::Subscriber<sensor_msgs::Image> Adepth_sub(nh, "/camera/aligned_depth_to_color/image_raw", q); 

  ros::Subscriber rgb_sub2 = nh.subscribe("/camera/color/image_raw", q, rgbCb); 
  ros::Subscriber dpt_sub2 = nh.subscribe("/camera/aligned_depth_to_color/image_raw", q, dptCb); 
  // message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth/image_rect_raw", q);
  // message_filters::Subscriber<sensor_msgs::Image> ir1_sub(nh, "/camera/infra1/image_rect_raw", q); 
  // message_filters::Subscriber<sensor_msgs::Image> ir2_sub(nh, "/camera/infra2/image_rect_raw", q); 

  // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> sync_pol;
  // message_filters::Synchronizer<sync_pol> sync(sync_pol(q), rgb_sub, Adepth_sub, depth_sub, ir1_sub, ir2_sub);
  // sync.registerCallback(boost::bind(&ImageGrabberFull,_1, _2, _3, _4, _5));
  
  // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
  // message_filters::Synchronizer<sync_pol> sync(sync_pol(q), rgb_sub, Adepth_sub);
  // sync.registerCallback(boost::bind(&ImageGrabberRGBD,_1, _2));
 
  ROS_WARN("realsense_wrapper.cpp: start to subscribe msgs!"); 

  ros::spin(); 
  ros::shutdown(); 

  return 0;
}

void rgbCb(const sensor_msgs::ImageConstPtr& msgRGB)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    // cv_bridge::CvImageConstPtr cv_ptrAD;

    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // imshow("Live Feed RGB:", cv_ptrRGB->image); 
    // waitKey(3); 
    
    // write images 
    stringstream tt;
    tt << msgRGB->header.stamp ;
    string tt_name = tt.str();
    string time_stamp = tt_name; //tt_name.substr (0,17);
    std_msgs::Header h_rgb = msgRGB->header;
    ROS_DEBUG("pub_rs_data: receive rgb msg: %s" , time_stamp.c_str()); 

    rgb_pub.publish(*(cv_ptrRGB.get())); 
    ros::spinOnce(); 
    return ; 
}
void dptCb(const sensor_msgs::ImageConstPtr& msgAD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrAD;

    try
    {
        cv_ptrAD = cv_bridge::toCvShare(msgAD, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // imshow("Live Feed RGB:", cv_ptrAD->image); 
    // waitKey(3); 
    
    // write images 
    stringstream tt;
    tt << msgAD->header.stamp ;
    string tt_name = tt.str();
    string time_stamp = tt_name; //tt_name.substr (0,17);

    ROS_WARN("pub_rs_data: receive dpt msg: %s" , time_stamp.c_str()); 

    dpt_pub.publish(*(cv_ptrAD.get())); 
    ros::spinOnce(); 
 
    return; 
}


void ImageGrabberRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgAD)
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

    // imshow("Live Feed RGB:", cv_ptrRGB->image); 
    // waitKey(3); 
    
    // write images 
    stringstream tt;
    tt << msgRGB->header.stamp ;
    string tt_name = tt.str();
    string time_stamp = tt_name; //tt_name.substr (0,17);
    // imwrite( gDir + "/" + gDataName + "/color/" + time_stamp+".png", cv_ptrRGB->image);
    // imwrite( gDir + "/" + gDataName + "/aligned_depth/" + tt_name+".png", cv_ptrAD->image);
    // imwrite( gDir + "/" + gDataName + "/depth/" + time_stamp+".png", cv_ptrAD->image);
    // imwrite( gDir + "/" + gDataName + "/ir/" + tt_name+".png", cv_ptrIr1->image);
    // imwrite( gDir + "/" + gDataName + "/ir2/" + tt_name+".png", cv_ptrIr2->image);

    std_msgs::Header h_rgb = msgRGB->header;
    std_msgs::Header h_d = msgAD->header; 
    // std_msgs::Header h_ir = msgIr1->header; 
    // std_msgs::Header h_ir2 = msgIr2->header; 

    stringstream ss;
    ss << h_rgb.stamp <<" "<<h_d.stamp;

    ROS_DEBUG("pub_rs_data: receive rgbd msg: %s" , ss.str().c_str()); 

    rgb_pub.publish(*(cv_ptrRGB.get())); 
    dpt_pub.publish(*(cv_ptrAD.get())); 
    ros::spinOnce(); 
 
    // stringstream ss;
    // ss << h_rgb.stamp << "  "<< "color/"<< h_rgb.stamp << ".png  "<<h_d.stamp << " aligned_depth/"<< h_rgb.stamp << ".png "<<h_d.stamp << " depth/"<< h_rgb.stamp << ".exr "<< h_ir.stamp<<" ir/"<<h_ir.stamp<<".png "<<h_ir2.stamp<<" ir2/"<<h_ir2.stamp<<".png";
    
    // ss << time_stamp << "  "<< "color/"<< time_stamp << ".png  "<<time_stamp << " depth/"<< time_stamp << ".png ";

    // string time_info = ss.str();
    // tsfile << time_info <<"\n";
    	

    return; 
}



void ImageGrabberFull(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgAD, const sensor_msgs::ImageConstPtr& msgD, const sensor_msgs::ImageConstPtr& msgIr1, const sensor_msgs::ImageConstPtr& msgIr2)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    cv_bridge::CvImageConstPtr cv_ptrAD;
    cv_bridge::CvImageConstPtr cv_ptrD;
    cv_bridge::CvImageConstPtr cv_ptrIr1; 
    cv_bridge::CvImageConstPtr cv_ptrIr2; 

    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB, sensor_msgs::image_encodings::BGR8);
        cv_ptrAD = cv_bridge::toCvShare(msgAD, sensor_msgs::image_encodings::TYPE_16UC1);
        cv_ptrD = cv_bridge::toCvShare(msgD, sensor_msgs::image_encodings::TYPE_32FC1);
        cv_ptrIr1 = cv_bridge::toCvShare(msgIr1, sensor_msgs::image_encodings::TYPE_8UC1); 
        cv_ptrIr2 = cv_bridge::toCvShare(msgIr2, sensor_msgs::image_encodings::TYPE_8UC1); 
        ROS_WARN("Receive msg rgb w %d h %d, dpt w %d h %d ir w %d h %d!", cv_ptrRGB->image.cols, cv_ptrRGB->image.rows, cv_ptrD->image.cols, cv_ptrD->image.rows, cv_ptrIr1->image.cols, cv_ptrIr1->image.rows); 
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    imshow("Live Feed RGB:", cv_ptrRGB->image); 
    waitKey(3); 
    
    // write images 
    // stringstream tt;
    // tt << msgRGB->header.stamp ;
    // string tt_name = tt.str();
    // string time_stamp = tt_name; //tt_name.substr (0,17);
    // imwrite( gDir + "/" + gDataName + "/color/" + time_stamp+".png", cv_ptrRGB->image);
    // imwrite( gDir + "/" + gDataName + "/aligned_depth/" + tt_name+".png", cv_ptrAD->image);
    // imwrite( gDir + "/" + gDataName + "/depth/" + time_stamp+".png", cv_ptrAD->image);
    // imwrite( gDir + "/" + gDataName + "/ir/" + tt_name+".png", cv_ptrIr1->image);
    // imwrite( gDir + "/" + gDataName + "/ir2/" + tt_name+".png", cv_ptrIr2->image);

    std_msgs::Header h_rgb = msgRGB->header;
    std_msgs::Header h_d = msgRGB->header; 
    std_msgs::Header h_ir = msgIr1->header; 
    std_msgs::Header h_ir2 = msgIr2->header; 
    
    rgb_pub.publish(*(cv_ptrRGB.get())); 
    dpt_pub.publish(*(cv_ptrAD.get())); 
    ros::spinOnce(); 
    // stringstream ss;
    // ss << h_rgb.stamp << "  "<< "color/"<< h_rgb.stamp << ".png  "<<h_d.stamp << " aligned_depth/"<< h_rgb.stamp << ".png "<<h_d.stamp << " depth/"<< h_rgb.stamp << ".exr "<< h_ir.stamp<<" ir/"<<h_ir.stamp<<".png "<<h_ir2.stamp<<" ir2/"<<h_ir2.stamp<<".png";
    
    // ss << time_stamp << "  "<< "color/"<< time_stamp << ".png  "<<time_stamp << " depth/"<< time_stamp << ".png ";

    // string time_info = ss.str();
    // tsfile << time_info <<"\n";

    return; 
}


