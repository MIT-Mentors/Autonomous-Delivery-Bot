#include "ros/ros.h"

#include "std_msgs/String.h"

#include <webots_ros/set_float.h>


std::string robotName{""};
int flag{0};

webots_ros::set_float leftWheelSrv;
webots_ros::set_float rightWheelSrv;

void callbackNameParser(const std_msgs::String::ConstPtr& model)
{
    ROS_INFO("Robot Name: [%s]", model->data.c_str());
    robotName = static_cast<std::string>(model->data.c_str());
    flag = 1;
}


int main(int argc, char **argv)
{
    ros::init(argc,argv,"main");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/model_name",1000,callbackNameParser);

    while (flag == 0)
        {ros::spinOnce();}

    //Setting position   

    double rightPos =  100.0;
    double leftPos = 100.0;

    ros::ServiceClient leftWheelClientPos = n.serviceClient<webots_ros::set_float>(robotName+"/left_wheel_motor/set_position");
    ros::ServiceClient rightWheelClientPos = n.serviceClient<webots_ros::set_float>(robotName+"/right_wheel_motor/set_position");

    leftWheelSrv.request.value = leftPos;
    rightWheelSrv.request.value = rightPos;

    if (!leftWheelClientPos.call(leftWheelSrv) || !rightWheelClientPos.call(rightWheelSrv) || !leftWheelSrv.response.success || !rightWheelSrv.response.success)
        {
            std::cout << "Failed to set position";
        }

    //Setting Velocities
    
    double left_velocity = 5.0;
    double right_velocity = 5.0;

    ros::ServiceClient leftWheelClient = n.serviceClient<webots_ros::set_float>(robotName+"/left_wheel_motor/set_velocity");
    ros::ServiceClient rightWheelClient = n.serviceClient<webots_ros::set_float>(robotName+"/right_wheel_motor/set_velocity");


    leftWheelSrv.request.value = left_velocity;
    rightWheelSrv.request.value = right_velocity;

    if (!leftWheelClient.call(leftWheelSrv) || !rightWheelClient.call(rightWheelSrv) || !leftWheelSrv.response.success || !rightWheelSrv.response.success)
        {
            std::cout << "Failed to set velocity";
        }
    
    return 0;
}
