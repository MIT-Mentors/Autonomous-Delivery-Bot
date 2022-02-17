#include <string>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <webots_ros/set_float.h>

int main(int argc, char **argv)
{
    ros::init(argc,argv,"delivery");
    ros::NodeHandle n;

    // Publish availability status

    ros::Publisher availability_pub = n.advertise<std_msgs::String>("availability",1000);

    std_msgs::String availability_status;
    std::stringstream ss_availability_status;
    ss_availability_status << "no";
    availability_status.data = ss_availability_status.str();
    
    ros::Rate loop_rate(100);

    while (ros::ok)
    {
        availability_pub.publish(availability_status);

        ros::spinOnce();
        loop_rate.sleep();
    }
}