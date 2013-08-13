#include <iostream>
#include <string>
#include <geometry_msgs/WrenchStamped.h>

#include "ros/ros.h"
#include <tf/transform_listener.h>
#include<tf/tf.h>
#include <geometry_msgs/PointStamped.h>



void Callback( const geometry_msgs::WrenchStamped::ConstPtr& data)
{

   double length_vector[3] = {1 , 2 , 3} ;
   float F[3]={0,0,0}, Fn[3] ;
   tf::Matrix3x3 rot_matrix;

   F[0]= data -> wrench.force.x;
   F[1]= data -> wrench.force.y;
   F[2]= data -> wrench.force.z;

   tf::TransformListener listener;

   ros::Rate rate(10.0);

   tf::StampedTransform transform;

   try{

       listener.lookupTransform("/kms400","/new_frame",ros::Time(0),transform);
       }
   catch (tf::TransformException ex)
       {


       ROS_ERROR("%s",ex.what());


       }

   rot_matrix=transform.getBasis();




   for(int j=0;j<3;j++)
   {
          Fn[j]=0;

          for(int k=0;k<3;k++)
           {

               Fn[j]=Fn[j]+rot_matrix[j][k] * F[k];

           }

   }

   std::cout<<"New Force Matrix"<<std::endl;
   for(int j=0;j<3;j++)
   {
       std::cout<<Fn[j]<<"/t";
   }


   std::cout<<"New Moment"<<std::endl;
   for(int j=0;j<3;j++)
   {
       std::cout<<Fn[j]*length_vector[j]<<"/t";

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
