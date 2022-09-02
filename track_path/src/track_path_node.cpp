#include "track_path/track_path.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Track_Path");
    Track_Path tp;
    ros::spin();
    return 0;
}
