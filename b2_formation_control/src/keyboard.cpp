#include <iostream>
#include <string>
#include <sstream>
#include <ros/ros.h>
#include <std_msgs/String.h>
using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "keyboard");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("keyboard", 1000);
    ros::Rate loop_rate(50);
    string ss;
    std_msgs::String msg;

    while (ros::ok())
    {   
        cin >> ss;
        if (ss == "q") {
            break;
        }
        msg.data = ss;
        ROS_INFO("%s", msg.data.c_str());
        chatter_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
