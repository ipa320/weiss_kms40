#include <iostream>
#include <string>
#include <geometry_msgs/WrenchStamped.h>

#include "ros/ros.h"
#include <tf/transform_listener.h>
#include<tf/tf.h>
#include <geometry_msgs/PointStamped.h>



void Callback( const geometry_msgs::WrenchStamped::ConstPtr& data)
{

   F[0]= data -> wrench.force.x;
   F[1]= data -> wrench.force.y;
   F[2]= data -> wrench.force.z;

   tf::TransformListener listener;

   ros::Rate rate(10.0);

   while(node.ok())
   {
       tf::StampedTransform transform;

       try{

           listener.lookupTransform("/kms400","/new_frame",ros::Time(0),transform);
       }
       catch (tf::TransformException ex)
       {

           ROS_ERROR("%s",ex.what());

       }

       rot_matrix=transform.getBasis();

   }


   for(int j=0;j<3;j++)
   {
          Fn[j]=0;

          for(int k=0;k<3;k++)
           {

               Fn[j]=Fn[j]+rot_matrix[j][k] * F[k];

           }

   }

   cout<<"New Force Matrix"<<std::endl;
   for(int j=0;j<3;j++)
   {
       cout<<Fn[j]<<"/t";
   }

}

int main(int argc, char **argv)
{

    ros::init( argc , argv , "Transform_Publisher");
    ros::NodeHandle n;




    ros::Subscriber sub =n.subscribe("/kms400",1000,Callback);
    ros::spin();

    return 0;

}
