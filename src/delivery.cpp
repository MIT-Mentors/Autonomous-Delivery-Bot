#include <array>
#include <string>
#include <iostream>
#include <vector>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Bool.h"

#include <webots_ros/set_float.h>
#include <geometry_msgs/Vector3.h>

ros::NodeHandle* nh; //pointer

std::string senderLocation;
std::string receiverLocation;

geometry_msgs:: Vector3 setpointsArray;


// double setpointsArray[4][3]{{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0}};

ros::Publisher availability_pub{};

// std::vector<std::array> setpointsArray{};

// double placeA[3]{-41.42,0.0,0.30};
double placeA[3]{-41.42,0.0,40.82};
double placeB[3]{-2.3,0,49.45};
double placeC[3]{58.59,0,40.82};
double placeD[3]{93.61,0,40.82};
double intersection[3]{-43,0.9,44};

double placeNil[3]{0.0,0.0,0.0};

bool reachedSetpointBool{0};

// double currLocation[3]{placeA};

// double array[3] places[5]{placeA,placeB,placeC,placeD,intersection};

// enum Place
// {
//     A;
//     B;
//     C;
//     D;
//     intersection;
//     numOfPlaces;
// };

void assignSetpoint(double placeArray[3])
{
    setpointsArray.x = placeArray[0];
    setpointsArray.y = placeArray[1];
    setpointsArray.z = placeArray[2];
}

void setAvailabilityStatus(std::string status)
{
    std_msgs::String availability_status;
    std::stringstream ss_availability_status;
    ss_availability_status << status;
    availability_status.data = ss_availability_status.str();
    availability_pub.publish(availability_status);
}

void senderLocationCallback(const std_msgs::String::ConstPtr &name)
{
    senderLocation = static_cast<std::string>(name->data.c_str());
}

void receiverLocationCallback(const std_msgs::String::ConstPtr &name)
{
    receiverLocation = static_cast<std::string>(name->data.c_str());
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

    availability_pub = n.advertise<std_msgs::String>("availability",1000);
    ros::Publisher setpoint_pub = n.advertise<geometry_msgs::Vector3>("setpoint",1000);

    // ros::Subscriber senderLocationSub = n.subscribe("sender_location",1000,senderLocationCallback);
    ros::Subscriber receiverLocationSub = n.subscribe("receiver_location",1000,receiverLocationCallback);
    ros::Subscriber reachedSetpointSub = n.subscribe("reachedSetpointBool",10,reachedSetpointCallback);

    ros::Rate loop_rate(100);

    // std_msgs::Int8 setpointFlag;

    std::string senderLocation{"Location A"}; 

    while (ros::ok())
    {

        if (senderLocation.compare("Location A") == 0)
        {
            assignSetpoint(placeA);
            setAvailabilityStatus("no");
        }
        else if (senderLocation.compare("Location B") == 0)
        {
            assignSetpoint(placeB);
            setAvailabilityStatus("no");
        }
        else if (senderLocation.compare("Location C") == 0)
        {
            assignSetpoint(placeC);
            setAvailabilityStatus("no");
        }
        else if (senderLocation.compare("Location D") == 0)
        {
            assignSetpoint(placeD);
            setAvailabilityStatus("no");
        }
        else
        {
            assignSetpoint(placeNil);
            setAvailabilityStatus("yes");
        }

        setpoint_pub.publish(setpointsArray);

        if (reachedSetpointBool)
            {
                if (receiverLocation.compare("Location A") == 0)
                {
                    assignSetpoint(placeA);
                }
                else if (receiverLocation.compare("Location B") == 0)
                {
                    assignSetpoint(placeB);
                }
                else if (receiverLocation.compare("Location C") == 0)
                {
                    assignSetpoint(placeC);
                }
                else if (receiverLocation.compare("Location D") == 0)
                {
                    assignSetpoint(placeD);
                }
                else
                {
                    assignSetpoint(placeNil);
                }
            }

        // if (senderLocation.compare("Location A"))
        // {
        //     setpointsArray[0][0] = placeA[0];
        //     setpointsArray[0][1] = placeA[1];
        //     setpointsArray[0][2] = placeA[2];
        // }

        // else if (senderLocation.compare("Location B"))
        // {
        //     setpointsArray[0][0] = intersection[0];
        //     setpointsArray[0][1] = intersection[1];
        //     setpointsArray[0][2] = intersection[2];

        //     setpointsArray[1][0] = placeB[0];
        //     setpointsArray[1][1] = placeB[1];
        //     setpointsArray[1][2] = placeB[2];
        // }
        // else if (senderLocation.compare("Location c"))
        // {
        //     setpointsArray[0][0] = intersection[0];
        //     setpointsArray[0][1] = intersection[1];
        //     setpointsArray[0][2] = intersection[2];

        //     setpointsArray[1][0] = placeC[0];
        //     setpointsArray[1][1] = placeC[1];
        //     setpointsArray[1][2] = placeC[2];
        // }
        // else if (senderLocation.compare("Location D"))
        // {
        //     setpointsArray[0][0] = intersection[0];
        //     setpointsArray[0][1] = intersection[1];
        //     setpointsArray[0][2] = intersection[2];

        //     setpointsArray[1][0] = placeD[0];
        //     setpointsArray[1][1] = placeD[1];
        //     setpointsArray[1][2] = placeD[2];
        // }

        // if (senderLocation.compare("Location A"))
        //     setpoint{placeA};
        // else if (senderLocation.compare("Location B"))
        //     setpoint{placeB};
        // else if (senderLocation.compare("Location C"))
        //     setpoint{placeC};
        // else if (senderLocation.compare("Location D"))
        //     setpoint{placeD};


        ros::spinOnce();
        loop_rate.sleep();
    }
}