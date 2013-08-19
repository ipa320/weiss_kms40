#include <iostream>
#include <string>
#include <geometry_msgs/WrenchStamped.h>

#include "ros/ros.h"
#include <tf/transform_listener.h>
#include<tf/tf.h>
#include <geometry_msgs/PointStamped.h>


/*tf::StampedTransform listener()
{
    tf::TransformListener listener;

    //ros::Rate rate(20.0);

    tf::StampedTransform transform;
     // Listen to kms40 to the tool frame
    try{

        listener.lookupTransform("/base_link","/ee_link",ros::Time(0),transform);
        }
    catch (tf::TransformException ex)
        {


        ROS_ERROR("%s",ex.what());


        }
    // Finding the rotation Matrix from kms40 to tool_tcp
    return transform;

}
void Callback( const geometry_msgs::WrenchStamped::ConstPtr& data)
{
//    float  F_tool[3], M_tool[3], F_tot[3] ;

   //double length_vector[3] = {1 , 2 , 3} ;

   tf::Matrix3x3 rot_matrix,trans_matrix;

   tf::Vector3 translation,Force_kms40, Moment_kms40 , F_tool , M_tool;

   Force_kms40[0]= data -> wrench.force.x;
   Force_kms40[1]= data -> wrench.force.y;
   Force_kms40[2]= data -> wrench.force.z;

    Moment_kms40[0]= data ->wrench.torque.x;
    Moment_kms40[1]= data ->wrench.torque.y;
    Moment_kms40[2]= data ->wrench.torque.z;

   tf::StampedTransform transform=listener();
   rot_matrix=transform.getBasis();

   //Finding the translation vector from kms40 to tool_tcp
   translation = transform.getOrigin()*rot_matrix;
   // Calculating the Force in the new tool_tcp
   F_tool = Force_kms40*rot_matrix;
   // Define the Tranlational matrix

          trans_matrix[0][0]= 0;
          trans_matrix[0][1]=-1*translation[2];
          trans_matrix[0][2]= translation[1];
          trans_matrix[1][0]= translation[2];
          trans_matrix[1][1]= 0;
          trans_matrix[1][2]=-1*translation[0];
          trans_matrix[2][0]=-1*translation[1];
          trans_matrix[2][1]= translation[0];
          trans_matrix[2][2]= 0;

    //Calculating the moment at the tool_tcp

   M_tool =     Force_kms40*(trans_matrix*rot_matrix)+Moment_kms40*rot_matrix;



   geometry_msgs::WrenchStamped msg;
   ros::NodeHandle r;
   ros::Publisher real_transform = r.advertise<geometry_msgs::WrenchStamped>("real_transform", 1000);
   msg.wrench.force.x = F_tool[0];
   msg.wrench.force.y = F_tool[1];
   msg.wrench.force.z = F_tool[2];
   msg.wrench.torque.x = M_tool[0];
   msg.wrench.torque.y = M_tool[1];
   msg.wrench.torque.z = M_tool[2];
    std::cout<<msg.wrench.force.x<<std::endl;
   std::cout<<msg.wrench.force.y<<std::endl;
   std::cout<<msg.wrench.force.z<<std::endl;
   std::cout<<msg.wrench.torque.x<<std::endl;
   std::cout<<msg.wrench.torque.y<<std::endl;
   std::cout<<msg.wrench.torque.z<<std::endl;
   real_transform.publish(msg);



}*/

int main(int argc, char **argv)
{
    ROS_INFO("starting the node");
    ros::init( argc , argv , "Transform_Publisher");

    ros::NodeHandle n;

        //ros::Subscriber sub =n.subscribe("/kms40",1000,Callback);

    // DEBUG
    tf::TransformListener listener;

    //ros::Rate rate(20.0);

    tf::StampedTransform transform;
     // Listen to kms40 to the tool frame
    try{

        ros::Time now = ros::Time::now();
            listener.waitForTransform("/base_link","/ee_link", now, ros::Duration(3.0));

        listener.lookupTransform("/base_link","/ee_link",ros::Time(0),transform);
        }
    catch (tf::TransformException ex)
        {


        ROS_ERROR("%s",ex.what());


        }


    ROS_INFO("publishing");
    ros::spin();

    return 0;

}
