#ifndef TRACK_PATH_H
#define TRACK_PATH_H
#include<ros/ros.h>
#include <unistd.h>

#include <iostream>
#include <vector>
#include <string>
#include <fstream> //ifstream 사용

#include <nav_msgs/Path.h>

#include<geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>

#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

class Track_Path{
private:
    void track_path_flag_CB(const std_msgs::Bool::ConstPtr &msg);

    void load_path();
    void load_data(std::ifstream &fin, std::vector<double>& vec);
    void offset_setting();
    void data_post_processing();


    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    ros::Publisher path_pub;
    ros::Publisher state_pub;
    ros::Publisher offset_pub;
    ros::Subscriber path_flag;


    ros::Rate loop_rate;

    bool flag_x, flag_y, flag;
    std::string utm_x_path, utm_y_path;
    double utm_x_offset, utm_y_offset;

    std::vector<double> wp_x;
    std::vector<double> wp_y;

    geometry_msgs::Pose utm_offset_pub;
    nav_msgs::Path global_path;
    std_msgs::String state;

public:
    Track_Path();
    void process();
};


#endif // TRACK_PATH_H
