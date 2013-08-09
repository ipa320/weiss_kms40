#include <iostream>
#include <string>


#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/WrenchStamped.h>


void transformCallback(const tf::tfMessage::ConstPtr& data)
{



    data->transforms.data();


}



int main(int argc, char **argv)
{

    ros::init( argc , argv , "Transform_Publisher");
    ros::NodeHandle n;

    ros::Subscriber transform = n.subscribe("/tf",1000,transformCallback);
    ros::spin();
    return 0;

}
