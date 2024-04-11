#include "ros/ros.h"
#include <sstream>
#include <std_msgs/Float32MultiArray.h>
#include <string> // Include <string> for std::string

std_msgs::Float32MultiArray Speeds;
float Ultrasonics_Readings[4] = {0};
std::string Direction;

void DirectionCallback(const std_msgs::Float32MultiArray::ConstPtr& msg_dir)
{
    if(msg_dir->data[0] >0)
    {
    	Direction="forward";
    }
    
    else
    {
    	Direction="backward";
    }
    
    if(msg_dir->data[1] >0)
    {
    	Direction="left";
    }
    
    else
    {
    	Direction="right";
    }
}

void UltrasonicCallback(const std_msgs::Float32MultiArray::ConstPtr& msg_dist)
{
	if( ( (Direction == "forward") && msg_dist[0] < 10 ) ||  Direction == "backward") && msg_dist[1] < 10 ) ||  Direction == "left") && msg_dist[2] < 10 ) ||  Direction == "right") && msg_dist[3] < 10 ) )
	{
            	Speeds.data.push_back(0.0);
        	Speeds.data.push_back(0.0);
        	Speeds.data.push_back(0.0);
        	
        	Stopping_pub.publish(Speeds);
        	
        	ros::SpinOnce();
        	loop.rate.sleep();
        }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Crash_Prevention");
    ros::NodeHandle n;
    ros::Rate loop_rate(1);
    
    // Publish to topics
    ros::Publisher Stopping_pub = nh.advertise<std_msgs::Float32MultiArray>("cmd_wheel", 1000);
    
    // Subscribe to topics
    ros::Subscriber Direction_Sub = n.subscribe("cmd_vel", 1000, DirectionCallback);
    ros::Subscriber Ultrasonic_Sub = n.subscribe("Ultrasonics", 1000, UltrasonicCallback);

    // Spin, waiting for callbacks
    ros::spin();

    return 0;
}

