#include <math.h>
#include <string>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>
#include <geometry_msgs/PointStamped.h>


std::string robotName{""};
std::string direction{""};
int flag{0};
int straightFlag{0};

double GPSValIntersection[3] = {-43,0.9,44};

ros::NodeHandle* nh; //pointer

webots_ros::set_float leftWheelSrv;
webots_ros::set_float rightWheelSrv;


void callbackNameParser(const std_msgs::String::ConstPtr &model)
{
    ROS_INFO("Robot Name: [%s]", model->data.c_str());
    robotName = static_cast<std::string>(model->data.c_str());
    flag = 1;
}

/* void callbackDirections(const std_msgs::String::ConstPtr& input_direction)
{
    ROS_INFO("Received direction command: %s", input_direction->data.c_str());
    direction = static_cast<std::string>(input_direction->data.c_str());

    if (direction.compare("straight") == 0)
        {
            straightFlag = 1;
        }
    else
        {
            straightFlag = 0;
        }
}*/

void setPosition()

{
    //Setting position   

    double rightPos = 10000.0;
    double leftPos = 10000.0;

    ros::ServiceClient leftWheelClientPos = nh->serviceClient<webots_ros::set_float>(robotName+"/left_wheel_motor/set_position");
    ros::ServiceClient rightWheelClientPos = nh->serviceClient<webots_ros::set_float>(robotName+"/right_wheel_motor/set_position");

    leftWheelSrv.request.value = leftPos;
    rightWheelSrv.request.value = rightPos;

    if (!leftWheelClientPos.call(leftWheelSrv) || !rightWheelClientPos.call(rightWheelSrv) || !leftWheelSrv.response.success || !rightWheelSrv.response.success)
        {
            std::cout << "Failed to set position";
        }
}

void enableGPS()
{
    ros::ServiceClient gpsClient = nh->serviceClient<webots_ros::set_int>(robotName+"/gps/enable");
    webots_ros::set_int gpsSrv;
    gpsSrv.request.value = 1;
}

void GPSCallback(const geometry_msgs::PointStamped::ConstPtr &values)
{
    double currLocation[3] = {values->point.x,values->point.y,values->point.z};

    //double intersectionDistance =  
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

    nh = &n;    // assigning node handle object to global pointer

    ros::Subscriber sub = n.subscribe("/model_name",1000,callbackNameParser);
    

    while (flag == 0) {ros::spinOnce();}

    setPosition();
    enableGPS();

    // Gps
    ros::Subscriber gps_sub = n.subscribe(robotName+"/gps/values",1000,GPSCallback);

    //Setting Velocities 

    //ros::Subscriber dir_sub = n.subscribe("/directions",1000,callbackDirections);

    double left_velocity = 10.0;
    double right_velocity = 10.0;
    //double velocity = 0.0;

    ros::ServiceClient leftWheelClient = n.serviceClient<webots_ros::set_float>(robotName+"/left_wheel_motor/set_velocity");
    ros::ServiceClient rightWheelClient = n.serviceClient<webots_ros::set_float>(robotName+"/right_wheel_motor/set_velocity");
   
    ros::Rate loop_rate(100); //100 Hz

    while (ros::ok())
    {
        /* velocity = getVelocity();
        left_velocity = velocity;
        right_velocity = velocity; */

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
