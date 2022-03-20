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

bool isReachedSetPoint{0};

std::string senderLocation{};
std::string receiverLocation{};

geometry_msgs::Vector3 setpointArray;

enum class ProgressStatus
{
    done,
    in_progress,
    none,
};

enum class AvailabilityStatus
{
    yes,
    no,
};

void assign_setpoint(double placeArray[3])
{
    setpointArray.x = placeArray[0];
    setpointArray.y = placeArray[1];
    setpointArray.z = placeArray[2];
}

void set_availability_status(std::vector <std::string> availabilityStatusOptions, AvailabilityStatus status, ros::Publisher *availabilityPub)
{
    
    std::stringstream strStreamAvailabilityStatus;
    strStreamAvailabilityStatus << availabilityStatusOptions[static_cast<int>(status)];

    std_msgs::String availabilityStatus;
    availabilityStatus.data = strStreamAvailabilityStatus.str();

    availabilityPub->publish(availabilityStatus);
}

void set_progress_status(std::vector<std::string> progressStatusOptions, ProgressStatus status, ros::Publisher *progressStatusPub)
{
    std::stringstream strStreamProgressStatus;
    strStreamProgressStatus << progressStatusOptions[static_cast<int>(status)];

    std_msgs::String progressStatus;
    progressStatus.data = strStreamProgressStatus.str();

    progressStatusPub->publish(progressStatus);
}

bool find_setpoint(std::string location)
{

    // Hardcoded GPS values
    static double placeA[3]{-41.42,0.0,40.82};
    static double placeB[3]{-2.3,0,49.45};
    static double placeC[3]{58.59,0,40.82};
    static double placeD[3]{93.61,0,40.82};
    static double placeNil[3]{100000.0,100000.0,100000.0}; // default

    if (location.compare("Location A") == 0)
    {
        assign_setpoint(placeA);
    }
    else if (location.compare("Location B") == 0)
    {
        assign_setpoint(placeB);
    }
    else if (location.compare("Location C") == 0)
    {
        assign_setpoint(placeC);
    }
    else if (location.compare("Location D") == 0)
    {
        assign_setpoint(placeD);
    }
    else
    {
        assign_setpoint(placeNil);
        return 0;
    }

    return 1;
}

void sender_location_callback(const std_msgs::String::ConstPtr &name)
{
    senderLocation = name->data.c_str();
}

void receiver_location_callback(const std_msgs::String::ConstPtr &name)
{
    receiverLocation = name->data.c_str();
}

void reached_setpoint_callback(const std_msgs::Bool::ConstPtr &value)
{   
    isReachedSetPoint = value->data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "delivery");
    ros::NodeHandle n;

    // Publishers
    ros::Publisher availabilityPub = n.advertise<std_msgs::String>("availability", 1000);
    ros::Publisher progressStatusPub = n.advertise<std_msgs::String>("progress", 1000);
    ros::Publisher setpointPub = n.advertise<geometry_msgs::Vector3>("setpoint", 1000);

    // Subscribers
    ros::Subscriber senderLocationSub = n.subscribe("senderLocation", 1000, sender_location_callback);
    ros::Subscriber receiverLocationSub = n.subscribe("receiverLocation", 1000, receiver_location_callback);
    ros::Subscriber reachedSetpointSub = n.subscribe("isReachedSetPoint", 10, reached_setpoint_callback);

    std::vector<std::string> progressStatusOptions {"done", "in progress", "none"};
    std::vector<std::string> availabilityStatusOptions {"yes", "no"};

    ros::Rate loopRate(10); 

    while (ros::ok())
    {
        // Sender loop
        while (ros::ok())
        {
            bool isfoundSetpoint{find_setpoint(senderLocation)};            
            setpointPub.publish(setpointArray);

            if (isfoundSetpoint)
            {
                set_availability_status(availabilityStatusOptions, AvailabilityStatus::no, &availabilityPub);
                set_progress_status(progressStatusOptions, ProgressStatus::in_progress, &progressStatusPub);
            }

            sleep(2);
            ros::spinOnce();
            loopRate.sleep();

            if (isReachedSetPoint)
            {
                ROS_INFO("Reached sender at %s", senderLocation.c_str());
                break;
            }
        }

        // Receiver loop
        while (ros::ok())
        {
            bool isfoundSetpoint{find_setpoint(receiverLocation)};
            setpointPub.publish(setpointArray);

            sleep(2);
            ros::spinOnce();
            loopRate.sleep();

            if (isReachedSetPoint)
            {
                set_progress_status(progressStatusOptions, ProgressStatus::done, &progressStatusPub);
                set_availability_status(availabilityStatusOptions, AvailabilityStatus::yes, &availabilityPub);
                ROS_INFO("Reached receiver at %s", receiverLocation.c_str());
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