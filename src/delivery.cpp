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

enum class ProgressStatusOptions
{
    done,
    in_progress,
    none,
};

enum class AvailabilityStatusOptions
{
    yes,
    no,
};

class Sender
{
public:
    std::string m_location{};
    ros::Subscriber m_locationSub{};

    Sender(ros::NodeHandle* n)
    {
        m_locationSub = n->subscribe("senderLocation", 1000, &Sender::location_callback, this);
    }

    void location_callback(const std_msgs::String::ConstPtr &name)
    {
        this->m_location = name->data.c_str();
    }
};

class Receiver
{
public:
    std::string m_location{};
    ros::Subscriber m_locationSub{};

    Receiver(ros::NodeHandle* n)
    {
        m_locationSub = n->subscribe("receiverLocation", 1000, &Receiver::location_callback, this);
    }

    void location_callback(const std_msgs::String::ConstPtr &name)
    {
        m_location = name->data.c_str();
    }
};

class SetPoint
{
public:
    bool m_isReachedSetPoint{0};  
    ros::NodeHandle* m_nodehandle{};
    ros::Publisher m_setpointPub{};
    ros::Subscriber m_reachedSetpointSub{};
    geometry_msgs::Vector3 setpointArray{};

    SetPoint(ros::NodeHandle* n)
    {
        m_nodehandle = n;
        m_setpointPub = n->advertise<geometry_msgs::Vector3>("setpoint", 1000);
        m_reachedSetpointSub = n->subscribe("isReachedSetPoint", 10, &SetPoint::reached_setpoint_callback, this);
    }

    void publish_setpoint(double placeArray[3])
    {
        setpointArray.x = placeArray[0];
        setpointArray.y = placeArray[1];
        setpointArray.z = placeArray[2];

        m_setpointPub.publish(setpointArray);
    }

    void reached_setpoint_callback(const std_msgs::Bool::ConstPtr &value)
    {   
        m_isReachedSetPoint = value->data;
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
            publish_setpoint(placeA);
        }
        else if (location.compare("Location B") == 0)
        {
            publish_setpoint(placeB);
        }
        else if (location.compare("Location C") == 0)
        {
            publish_setpoint(placeC);
        }
        else if (location.compare("Location D") == 0)
        {
            publish_setpoint(placeD);
        }
        else
        {
            publish_setpoint(placeNil);
            return 0;
        }

        return 1;
    }
};

class BotAvailability
{
public:
    ros::NodeHandle* m_nodehandle{};
    ros::Publisher m_availabilityPub{};
    std::vector<std::string> m_availabilityStatusOptions{};

    BotAvailability(ros::NodeHandle* n)
    {
        m_nodehandle = n;
        m_availabilityPub = n->advertise<std_msgs::String>("availability", 1000);
        m_availabilityStatusOptions = {"yes", "no"};
    }

    void set_availability_status(AvailabilityStatusOptions status)
    {
        std::stringstream strStreamAvailabilityStatus;
        strStreamAvailabilityStatus << m_availabilityStatusOptions[static_cast<int>(status)];

        std_msgs::String availabilityStatus;
        availabilityStatus.data = strStreamAvailabilityStatus.str();

        m_availabilityPub.publish(availabilityStatus);
    }
};

class ProgressStatus
{
public:
    ros::NodeHandle* m_nodehandle{};
    ros::Publisher m_progressStatusPub{};
    std::vector<std::string> m_progressStatusOptions{};

    ProgressStatus(ros::NodeHandle* n)
    {
        m_nodehandle = n;
        m_progressStatusPub = n->advertise<std_msgs::String>("progress", 1000);
        m_progressStatusOptions = {"done", "in progress", "none"};
    }

    void set_progress_status(ProgressStatusOptions status)
    {
        std::stringstream strStreamProgressStatus;
        strStreamProgressStatus << m_progressStatusOptions[static_cast<int>(status)];

        std_msgs::String progressStatus;
        progressStatus.data = strStreamProgressStatus.str();

        m_progressStatusPub.publish(progressStatus);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "delivery");
    ros::NodeHandle n;

    BotAvailability botAvailability{&n};
    ProgressStatus progressStatus{&n};
    SetPoint setpoint{&n};
    Sender sender{&n};
    Receiver receiver{&n};

    ros::Rate loopRate(10); 

    while (ros::ok())
    {
        // Sender loop
        while (ros::ok())
        {
            bool isfoundSetpoint{setpoint.find_setpoint(sender.m_location)};            

            if (isfoundSetpoint)
            {
                botAvailability.set_availability_status(AvailabilityStatusOptions::no);
                progressStatus.set_progress_status(ProgressStatusOptions::in_progress);
            }

            sleep(2);
            ros::spinOnce();
            loopRate.sleep();

            if (setpoint.m_isReachedSetPoint)
            {
                ROS_INFO("Reached sender at %s", sender.m_location.c_str());
                break;
            }
        }

        // Receiver loop
        while (ros::ok())
        {
            bool isfoundSetpoint{setpoint.find_setpoint(receiver.m_location)};

            sleep(2);
            ros::spinOnce();
            loopRate.sleep();

            if (setpoint.m_isReachedSetPoint)
            {
                progressStatus.set_progress_status(ProgressStatusOptions::done);
                botAvailability.set_availability_status(AvailabilityStatusOptions::yes);

                ROS_INFO("Reached receiver at %s", receiver.m_location.c_str());
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