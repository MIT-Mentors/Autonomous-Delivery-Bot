#include <array>
#include <iostream>
#include <string>
#include <vector>

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"

#include <webots_ros/set_float.h>
#include <geometry_msgs/Vector3.h>

// For sleep function
#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

// Hardcoded GPS values
double placeA[3]{-41.42,0.0,40.82};
double placeB[3]{-2.3,0,49.45};
double placeC[3]{58.59,0,40.82};
double placeD[3]{93.61,0,40.82};
double placeNil[3]{100000.0,100000.0,100000.0}; // default

bool reachedSetpointBool{0};

std::string senderLocation{};
std::string receiverLocation{};

geometry_msgs:: Vector3 setpointArray;

ros::Publisher availabilityPub{};
ros::Publisher progressStatusPub{};
ros::Publisher setpointPub{};

void assignSetpoint(double placeArray[3])
{
    setpointArray.x = placeArray[0];
    setpointArray.y = placeArray[1];
    setpointArray.z = placeArray[2];
}

void setAvailabilityStatus(std::string status)
{
    std_msgs::String availabilityStatus;
    std::stringstream ssAvailabilityStatus;
    ssAvailabilityStatus << status;
    availabilityStatus.data = ssAvailabilityStatus.str();

    availabilityPub.publish(availabilityStatus);
}

void setProgressStatus(std::string status)
{
    std_msgs::String progressStatus;
    std::stringstream ss_progressStatus;
    ss_progressStatus << status;
    progressStatus.data = ss_progressStatus.str();

    progressStatusPub.publish(progressStatus);
}

void senderLocationCallback(const std_msgs::String::ConstPtr &name)
{
    senderLocation = name->data.c_str();
}

void receiverLocationCallback(const std_msgs::String::ConstPtr &name)
{
    receiverLocation = name->data.c_str();
}

void reachedSetpointCallback(const std_msgs::Bool::ConstPtr &value)
{   
    reachedSetpointBool = value->data;
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"delivery");
    ros::NodeHandle n;

    // Publish availability status

    availabilityPub = n.advertise<std_msgs::String>("availability",1000);
    progressStatusPub = n.advertise<std_msgs::String>("progress",1000);
    setpointPub = n.advertise<geometry_msgs::Vector3>("setpoint",1000);

    // Subscribers
    ros::Subscriber senderLocationSub = n.subscribe("senderLocation",1000,senderLocationCallback);
    ros::Subscriber receiverLocationSub = n.subscribe("receiverLocation",1000,receiverLocationCallback);
    ros::Subscriber reachedSetpointSub = n.subscribe("reachedSetpointBool",1000,reachedSetpointCallback);

    ros::Rate loopRate(10); 

    while (ros::ok())
    {
        // Sender loop
        int senderloopCount{0};
        while (ros::ok())
        {
            bool foundSetpointBool{findSetpoint(senderLocation)};            
            setpointPub.publish(setpointArray);

            if (foundSetpointBool)
            {
                setAvailabilityStatus("no");
                setProgressStatus("in progress");
            }

            std::cout << "Setting setpoint to " << senderLocation << '\n';
            setpointPub.publish(setpointArray);

            if (loopCount1 == 0)
            {
                sleep(2);
                ++loopCount1;
                ros::spinOnce();
                loopRate.sleep();
            }
            
            ros::spinOnce();
            loopRate.sleep();

            if (reachedSetpointBool)
            {
                ROS_INFO("Reached sender at %s",senderLocation.c_str());
                break;
            }
        }

        // Receiver loop
        int receiverloopCount{0};
        while (ros::ok())
        {
            bool foundSetpointBool{findSetpoint(receiverLocation)};
            setpointPub.publish(setpointArray);

            setpointPub.publish(setpointArray);

            if (loopCount == 0)
            {
                sleep(2);
                ++loopCount;
                ros::spinOnce();
                loopRate.sleep();
            }
            
            ros::spinOnce();
            loopRate.sleep();

            if (reachedSetpointBool)
            {
                setProgressStatus("done");
                setAvailabilityStatus("yes");
                ros::spinOnce();
                loopRate.sleep();
                sleep(2);
                break;
            }
        }

        ros::spinOnce();
        loopRate.sleep();
    }
}