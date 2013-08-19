#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <tf/transform_listener.h>

#include <boost/thread.hpp>

class WrenchTransformer
{
protected:
    ros::NodeHandle _nh;
    ros::Subscriber _wrenchSub;
    ros::Publisher _wrenchPub;
    tf::TransformListener _tfListener;

    geometry_msgs::WrenchStamped _transWrench;
    boost::shared_mutex _wrenchDataAccess;

    std::string transFrameId;

    void kms40Callback(const geometry_msgs::WrenchStampedConstPtr& data)
    {
        geometry_msgs::WrenchStamped transWrench;
        transWrench.header.stamp = data->header.stamp;
        transWrench.header.frame_id = transFrameId;

        if (data->header.frame_id.compare(transFrameId) != 0)
        {


            tf::Matrix3x3 rot_matrix,trans_matrix;

            tf::Vector3 translation,Force_kms40, Moment_kms40 , F_tool , M_tool;

            Force_kms40[0]= data -> wrench.force.x;
            Force_kms40[1]= data -> wrench.force.y;
            Force_kms40[2]= data -> wrench.force.z;

             Moment_kms40[0]= data ->wrench.torque.x;
             Moment_kms40[1]= data ->wrench.torque.y;
             Moment_kms40[2]= data ->wrench.torque.z;


            tf::TransformListener listener;

            ros::Rate rate(20.0);

            tf::StampedTransform transform;
             // Listen to kms40 to the tool frame
            try{
                ros::Time now = ros::Time::now();
                listener.waitForTransform("/upper_arm_link","/forearm_link", now, ros::Duration(2.0));
                listener.lookupTransform("/upper_arm_link","/forearm_link",ros::Time(0),transform);
                }
            catch (tf::TransformException ex)
                {


                ROS_ERROR("%s",ex.what());


                }
            // Finding the rotation Matrix from kms40 to tool_tcp
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

            transWrench.wrench.force.x = F_tool[0];
            transWrench.wrench.force.y = F_tool[1];
            transWrench.wrench.force.z = F_tool[2];
            transWrench.wrench.torque.x = M_tool[0];
            transWrench.wrench.torque.y = M_tool[1];
            transWrench.wrench.torque.z = M_tool[2];



        }
        else
        {
            transWrench.wrench = data->wrench;
        }

        // get upgradable access
        boost::upgrade_lock<boost::shared_mutex> lock(_wrenchDataAccess);

        // get exclusive access
        boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);

        _transWrench = transWrench;



    }

    void getWrench(geometry_msgs::WrenchStamped& wrench)
    {
        boost::shared_lock<boost::shared_mutex> lock(_wrenchDataAccess);
        wrench = _transWrench;
    }


public:
    WrenchTransformer(std::string name)
    {
        _wrenchPub = _nh.advertise<geometry_msgs::WrenchStamped>("/kms40_transformed", 1);

        _wrenchSub = _nh.subscribe("/kms40", 1, &WrenchTransformer::kms40Callback, this);

        if( !ros::param::get("~transformFrameId", transFrameId) )
        {
            ROS_ERROR("Cannot find transformFrameId @ parameterServer");
            transFrameId = "transform";
            //_nh.shutdown();
        }

        double frequency = 500.0;
        if( !ros::param::get("~publishingRate", frequency) )
        {
            ROS_ERROR("Cannot find publishingRate @ parameterServer, using default 50Hz");
        }

        ros::Rate r(frequency);

        geometry_msgs::WrenchStamped wrench;

        while (ros::ok())
        {
            getWrench(wrench);

            _wrenchPub.publish(wrench);

            ros::spinOnce();


        }

    }

};



int main( int argc, char** argv )
{
    ros::init(argc, argv, "WrenchTransformer");

//    ROS_ERROR("Node not working as hoped. Math failed");
    WrenchTransformer wrenchTransformer(ros::this_node::getName());

    return 0;
}
