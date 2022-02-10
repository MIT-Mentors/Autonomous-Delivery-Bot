#include "ros/ros.h"
#include "std_msgs/String.h"
#include <typeinfo>

#include <webots_ros/set_float.h>


std::string robotName{""};
int flag{0};

ros::ServiceClient leftWheelClient;
webots_ros::set_float leftWheelSrv;

ros::ServiceClient rightWheelClient;
webots_ros::set_float rightWheelSrv;

void callbackNameParser(const std_msgs::String::ConstPtr& model)
{
    //std::cerr << flag << '\n';
    ROS_INFO("I heard: [%s]", model->data.c_str());
    robotName = static_cast<std::string>(model->data.c_str());
    flag = 1;
    std::cout << robotName << '\n';
}

void initNavigation(ros::NodeHandle n)
{
    std::cout << "successfully called init" << '\n';

}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"main");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/model_name",1000,callbackNameParser);

    double left_velocity = 5.0;
    double right_velocity = 5.0;

    double rPosition =  100.0;
    double lPosition = 100.0;

    while (true)
    {
        if (flag == 1)
             {
                initNavigation(n);
                break;
             }
        
        ros::spinOnce();
    }

    ros::ServiceClient leftWheelClientPos = n.serviceClient<webots_ros::set_float>(robotName+"/left_wheel_motor/set_position");
    ros::ServiceClient rightWheelClientPos = n.serviceClient<webots_ros::set_float>(robotName+"/right_wheel_motor/set_position");

    leftWheelSrv.request.value = lPosition;
    rightWheelSrv.request.value = rPosition;

    if (!leftWheelClientPos.call(leftWheelSrv) || !rightWheelClientPos.call(rightWheelSrv) || !leftWheelSrv.response.success || !rightWheelSrv.response.success)
        {
            std::cout << "Failed to set position";
        }

    leftWheelClient = n.serviceClient<webots_ros::set_float>(robotName+"/left_wheel_motor/set_velocity");
    rightWheelClient = n.serviceClient<webots_ros::set_float>(robotName+"/right_wheel_motor/set_velocity");


    leftWheelSrv.request.value = left_velocity;
    rightWheelSrv.request.value = right_velocity;

    if (!leftWheelClient.call(leftWheelSrv) || !rightWheelClient.call(rightWheelSrv) || !leftWheelSrv.response.success || !rightWheelSrv.response.success)
        {
            std::cout << "Failed to set velocity";
        }
    
    while (ros::ok())
    {}

    return 0;
}
