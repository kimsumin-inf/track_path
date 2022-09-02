#include "track_path/track_path.h"

using namespace std;

Track_Path::Track_Path()
:nh(""), pnh(""), loop_rate(10)
{
    // txt 파일 경로 param 화
    pnh.param<string>("utm_x_path", utm_x_path, "/home/sumin/catkin_ws/src/track_map_generate/path/utm_x.txt");
    pnh.param<string>("utm_y_path", utm_y_path, "/home/sumin/catkin_ws/src/track_map_generate/path/utm_y.txt");


    path_pub = nh.advertise<nav_msgs::Path>("/track/global_path",1);
    state_pub = nh.advertise<std_msgs::String>("/track/state",1);
    offset_pub= nh.advertise<geometry_msgs::Pose>("/track/utm_offset",1);

    path_flag = nh.subscribe("/Track_Path/Exist",1, &Track_Path::track_path_flag_CB, this );

    flag = false;
    flag_x = (access(utm_x_path.c_str(),F_OK)==0);
    flag_y = (access(utm_y_path.c_str(),F_OK)==0);





}

void Track_Path::track_path_flag_CB(const std_msgs::Bool::ConstPtr &msg) {
    flag = (*msg).data;

    if (flag ==true){
        if ( (access(utm_x_path.c_str(),F_OK)==0) )
            flag_x = true; // utm_x 파일 존재 유무
        if ((access(utm_y_path.c_str(),F_OK)==0))
            flag_y = true; // utm_y 파일 존재 유무

        load_path();
        offset_setting();
        data_post_processing();
        process();
        ros::Duration(0.3).sleep();
        loop_rate.sleep();
    }
    else {
        cout << "flag_x: "<< flag_x << endl;
        cout << "flag_y: "<< flag_y << endl;
        cout << "flag: "<< flag << endl;
    }
}

void Track_Path::load_path()  {
    ifstream fin_x, fin_y;

    fin_x.open(utm_x_path);
    fin_y.open(utm_y_path);

    load_data(fin_x, wp_x);
    load_data(fin_y, wp_y);

    fin_x.close();
    fin_y.close();
}

void Track_Path::load_data(std::ifstream &fin, std::vector<double>& vec) {
    string lines;
    vec.clear();
    if (fin.is_open()){
        while(getline(fin, lines)){
            double data = std::stod(lines.c_str());
            vec.push_back(data);
        }
    }
    else {
        cerr <<"can't load path data"<<endl;
    }
}

void Track_Path::offset_setting() {
    utm_x_offset = wp_x.at(0);
    utm_y_offset = wp_y.at(0);
    utm_offset_pub.position.x = utm_x_offset;
    utm_offset_pub.position.y = utm_y_offset;
    utm_offset_pub.position.z =0;
    offset_pub.publish(utm_offset_pub);
}

void Track_Path::data_post_processing() {
    for (auto& i : wp_x){
        i -= utm_x_offset;
    }
    for (auto& i : wp_y){
        i -= utm_y_offset;
    }
}


void Track_Path::process() {

    if (flag_x && flag_y && flag){
        ROS_INFO("Global Path Published");
        global_path.header.stamp = ros::Time::now();
        global_path.header.frame_id = "map";
        global_path.poses.clear();

        geometry_msgs::PoseStamped temp_pose;
        temp_pose.header.stamp = ros::Time::now();
        temp_pose.header.frame_id = "map";

        for(int i = 0; i < wp_x.size(); i++)
        {
            temp_pose.pose.position.x = wp_x.at(i);
            temp_pose.pose.position.y = wp_y.at(i);
            temp_pose.pose.position.z = 0;

            global_path.poses.push_back(temp_pose);
        }
        path_pub.publish(global_path);
        global_path.poses.clear();

        flag_x = false;
        flag_y = false;
    }
    else {
        ROS_INFO("flag error");

    }

    std::stringstream state_stream;
    std::string temp_state;

    temp_state = "no";

    state_stream << temp_state;
    state.data = state_stream.str();

    state_pub.publish(state);
}