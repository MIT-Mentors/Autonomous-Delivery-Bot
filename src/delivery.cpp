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
    std::stringstream ssProgressStatus;
    ssProgressStatus << status;
    progressStatus.data = ssProgressStatus.str();

    progressStatusPub.publish(progressStatus);
}

bool findSetpoint(std::string location)
{
    if (location.compare("Location A") == 0)
    {
        assignSetpoint(placeA);
    }
    else if (location.compare("Location B") == 0)
    {
        assignSetpoint(placeB);
    }
    else if (location.compare("Location C") == 0)
    {
        assignSetpoint(placeC);
    }
    else if (location.compare("Location D") == 0)
    {
        assignSetpoint(placeD);
    }
    else
    {
        assignSetpoint(placeNil);
        return 0;
    }

    return 1;
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

    // Publishers
    availabilityPub = n.advertise<std_msgs::String>("availability",1000);
    progressStatusPub = n.advertise<std_msgs::String>("progress",1000);
    setpointPub = n.advertise<geometry_msgs::Vector3>("setpoint",1000);

    // Subscribers
    ros::Subscriber senderLocationSub = n.subscribe("senderLocation",1000,senderLocationCallback);
    ros::Subscriber receiverLocationSub = n.subscribe("receiverLocation",1000,receiverLocationCallback);
    ros::Subscriber reachedSetpointSub = n.subscribe("reachedSetpointBool",10,reachedSetpointCallback);

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

            if (senderloopCount++ == 0) {sleep(2);}
            
            ros::spinOnce();
            loopRate.sleep();

            if (reachedSetpointBool)
            {
                break;
            }
        }

        // Receiver loop
        int receiverloopCount{0};
        while (ros::ok())
        {
            bool foundSetpointBool{findSetpoint(receiverLocation)};
            setpointPub.publish(setpointArray);

            if (receiverloopCount++ == 0) {sleep(2);}

            ros::spinOnce();
            loopRate.sleep();

            if (reachedSetpointBool)
            {
                setProgressStatus("done");
                setAvailabilityStatus("yes");
                ROS_INFO("Reached receiver at %s",receiverLocation.c_str());
                ROS_INFO("Completed delivery!");
                ros::spinOnce();
                loopRate.sleep();
                sleep(2);
                break;
            }
        }

        sleep(2);
        ros::spinOnce();
        loopRate.sleep();
    }
}