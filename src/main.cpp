#include <string>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <webots_ros/set_float.h>


std::string robotName{""};
std::string direction{""};
int flag{0};
int straightFlag{0};

webots_ros::set_float leftWheelSrv;
webots_ros::set_float rightWheelSrv;

void callbackNameParser(const std_msgs::String::ConstPtr& model)
{
    ROS_INFO("Robot Name: [%s]", model->data.c_str());
    robotName = static_cast<std::string>(model->data.c_str());
    flag = 1;
}

void callbackDirections(const std_msgs::String::ConstPtr& input_direction)
{
    ROS_INFO("Direction: %s", input_direction->data.c_str());
    direction = static_cast<std::string>(input_direction->data.c_str());

    if (direction.compare("straight") == 0)
        {
            straightFlag = 1;
        }
    else
        {
            straightFlag = 0;
        }
}

double getVelocity()
{
    if (straightFlag)
        return 5.0;
    else
        return 0.0;
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"main");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/model_name",1000,callbackNameParser);

    while (flag == 0)
        {ros::spinOnce();}

    //Setting position   

    double rightPos = 10000.0;
    double leftPos = 10000.0;

    ros::ServiceClient leftWheelClientPos = n.serviceClient<webots_ros::set_float>(robotName+"/left_wheel_motor/set_position");
    ros::ServiceClient rightWheelClientPos = n.serviceClient<webots_ros::set_float>(robotName+"/right_wheel_motor/set_position");

    leftWheelSrv.request.value = leftPos;
    rightWheelSrv.request.value = rightPos;

    if (!leftWheelClientPos.call(leftWheelSrv) || !rightWheelClientPos.call(rightWheelSrv) || !leftWheelSrv.response.success || !rightWheelSrv.response.success)
        {
            std::cout << "Failed to set position";
        }


    //Setting Velocities

    
    ros::Rate loop_rate(1);

    ros::Subscriber dir_sub = n.subscribe("/directions",1000,callbackDirections);

    double left_velocity = 0.0;
    double right_velocity = 0.0;
    double velocity = 0.0;

    ros::ServiceClient leftWheelClient = n.serviceClient<webots_ros::set_float>(robotName+"/left_wheel_motor/set_velocity");
    ros::ServiceClient rightWheelClient = n.serviceClient<webots_ros::set_float>(robotName+"/right_wheel_motor/set_velocity");


    


    while (ros::ok())
    {
        velocity = getVelocity();
        left_velocity = velocity;
        right_velocity = velocity;

        leftWheelSrv.request.value = left_velocity;
        rightWheelSrv.request.value = right_velocity;

        if (!leftWheelClient.call(leftWheelSrv) || !rightWheelClient.call(rightWheelSrv) || !leftWheelSrv.response.success || !rightWheelSrv.response.success)
            {
                std::cout << "Failed to set velocity";
            }
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}
