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

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

ros::NodeHandle* nh; //pointer

std::string senderLocation;
std::string receiverLocation;

geometry_msgs:: Vector3 setpointsArray;


// double setpointsArray[4][3]{{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0}};

ros::Publisher availability_pub{};
ros::Publisher progress_status_pub{};

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

    placeNil[0] = placeArray[0];
    placeNil[1] = placeArray[1];
    placeNil[2] = placeArray[2];
}

void setAvailabilityStatus(std::string status)
{
    std_msgs::String availability_status;
    std::stringstream ss_availability_status;
    ss_availability_status << status;
    availability_status.data = ss_availability_status.str();

    availability_pub.publish(availability_status);
}

void setProgressStatus(std::string status)
{
    std_msgs::String progress_status;
    std::stringstream ss_progress_status;
    ss_progress_status << status;
    progress_status.data = ss_progress_status.str();

    progress_status_pub.publish(progress_status);
}

void senderLocationCallback(const std_msgs::String::ConstPtr &name)
{
    senderLocation = static_cast<std::string>(name->data.c_str());
    // std::cout << "Sender loc: " << senderLocation << '\n';
}

void receiverLocationCallback(const std_msgs::String::ConstPtr &name)
{
    receiverLocation = static_cast<std::string>(name->data.c_str());
    // std::cout << "Reeceiver loc: " << receiverLocation << '\n';

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
    progress_status_pub = n.advertise<std_msgs::String>("progress",1000);

    ros::Publisher setpoint_pub = n.advertise<geometry_msgs::Vector3>("setpoint",1000);

    ros::Subscriber senderLocationSub = n.subscribe("sender_location",1000,senderLocationCallback);
    ros::Subscriber receiverLocationSub = n.subscribe("receiver_location",1000,receiverLocationCallback);
    ros::Subscriber reachedSetpointSub = n.subscribe("reachedSetpointBool",10,reachedSetpointCallback);

    ros::Rate loop_rate(10); 

    while (ros::ok())
    {
        // std::cout << "reachedSetpointBool: " << reachedSetpointBool << '\n';
        while (ros::ok())
        {
            int loopCount1{0};

            if (senderLocation.compare("Location A") == 0)
            {
                assignSetpoint(placeA);
                setAvailabilityStatus("no");
                setProgressStatus("in progress");
            }
            else if (senderLocation.compare("Location B") == 0)
            {
                assignSetpoint(placeB);
                setAvailabilityStatus("no");
                setProgressStatus("in progress");
            }
            else if (senderLocation.compare("Location C") == 0)
            {
                assignSetpoint(placeC);
                setAvailabilityStatus("no");
                setProgressStatus("in progress");
            }
            else if (senderLocation.compare("Location D") == 0)
            {
                assignSetpoint(placeD);
                setAvailabilityStatus("no");
                setProgressStatus("in progress");
            }
            else
            {
                // assignSetpoint(placeNil);
            }

            std::cout << "Setting setpoint to " << senderLocation << '\n';
            setpoint_pub.publish(setpointsArray);

            if (loopCount1 == 0)
            {
                sleep(2);
                ++loopCount1;
                ros::spinOnce();
                loop_rate.sleep();
            }
            
            ros::spinOnce();
            loop_rate.sleep();

            if (reachedSetpointBool)
            {
                break;
            }
        }

        int loopCount{0};

        while (ros::ok())
        {
            if (receiverLocation.compare("Location A") == 0)
            {
                assignSetpoint(placeA);
                // setAvailabilityStatus("no");
                // setProgressStatus("in progress");
            }
            else if (receiverLocation.compare("Location B") == 0)
            {
                assignSetpoint(placeB);
                // setAvailabilityStatus("no");
                // setProgressStatus("in progress");
            }
            else if (receiverLocation.compare("Location C") == 0)
            {
                assignSetpoint(placeC);
                // setAvailabilityStatus("no");
                // setProgressStatus("in progress");
            }
            else if (receiverLocation.compare("Location D") == 0)
            {
                assignSetpoint(placeD);
                // setAvailabilityStatus("no");
                // setProgressStatus("in progress");
            }
            else
            {
                // assignSetpoint(placeNil);
            }

            std::cout << "Seeting setpoint to " << receiverLocation << '\n';

            setpoint_pub.publish(setpointsArray);

            if (loopCount == 0)
            {
                sleep(2);
                ++loopCount;
                ros::spinOnce();
                loop_rate.sleep();
            }
            
            ros::spinOnce();
            loop_rate.sleep();

            if (reachedSetpointBool)
            {
                setProgressStatus("done");
                std::cout << "Seeting progress status to done\n";
                setAvailabilityStatus("yes");
                break;
            }
        }

        // break;
        sleep(2);
        ros::spinOnce();
        loop_rate.sleep();
    }
}