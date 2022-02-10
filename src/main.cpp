#include "ros/ros.h"
#include "std_msgs/String.h"

void callbackNameParser(const std_msgs::String::ConstPtr& model)
{
    ROS_INFO("I heard: [%s]", model->data.c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"main");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/model_name",1000,callbackNameParser);

    ros::spin();

    return 0;
}
