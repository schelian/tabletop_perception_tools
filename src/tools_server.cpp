#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tools_server");
    ros::NodeHandle node("~");

    while (ros::ok())
    {
        ros::spinOnce();
    }
    return 0;
}

