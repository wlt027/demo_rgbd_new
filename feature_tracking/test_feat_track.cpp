/*
    test by images 
*/

#include <ros/ros.h>
#include <string>
#include <iostream>
#include "feature_tracker.h"

using namespace std; 

void test_feat_track(); 

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "test_feat_track"); 
    ros::NodeHandle nh; 
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    test_feat_track(); 
    return 1; 
}

void test_feat_track()
{	
    string dir = "/home/davidz/work/data/drone/dataset_1/color";
    string img1 = dir + string("/1532020870.242634.png"); 
    string img2 = dir + string("/1532020870.410202.png"); 
    string img3 = dir + string("/1532020870.544670.png"); 
    
    CFeatureTracker tracker; 
    cv::Mat img = cv::imread(img1.c_str(), -1); 
    cv::Mat greyImg; 
    cv::cvtColor(img, greyImg, CV_RGB2GRAY); 
    tracker.handleImage(greyImg); 
    img = cv::imread(img2.c_str(), -1); 
    cv::cvtColor(img, greyImg, CV_RGB2GRAY); 

    tracker.handleImage(greyImg); 
    img = cv::imread(img3.c_str(), -1); 
    cv::cvtColor(img, greyImg, CV_RGB2GRAY); 
    tracker.handleImage(greyImg); 
    
    cout <<"Done!"<<endl; 
}



