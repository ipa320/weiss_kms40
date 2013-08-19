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

    void kms40Callback(const geometry_msgs::WrenchStampedConstPtr& msg)
    {
        geometry_msgs::WrenchStamped transWrench;
        transWrench.header.stamp = msg->header.stamp;
        transWrench.header.frame_id = transFrameId;

        if (msg->header.frame_id.compare(transFrameId) != 0)
        {
            tf::Transform wrenchTransMsg, wrenchTransWanted;

            wrenchTransMsg.setOrigin(tf::Vector3(msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z));
            wrenchTransMsg.setRotation(tf::createQuaternionFromRPY(msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z));

            try
            {
                tf::StampedTransform transform;

                _tfListener.lookupTransform(transFrameId, msg->header.frame_id, ros::Time(0), transform);
//                _tfListener.lookupTransform(transFrameId, msg->header.frame_id, msg->header.stamp, transform);

                wrenchTransWanted = transform.inverse() * wrenchTransMsg * transform;

                transWrench.wrench.force.x = wrenchTransWanted.getOrigin().x();
                transWrench.wrench.force.y = wrenchTransWanted.getOrigin().y();
                transWrench.wrench.force.z = wrenchTransWanted.getOrigin().z();
                wrenchTransWanted.getBasis().getRPY(transWrench.wrench.torque.x, transWrench.wrench.torque.y, transWrench.wrench.torque.z);

                ROS_INFO("tf transformation done!");
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("tf transformation failed. Canceling Action!");
                ROS_ERROR("tf::Transformation Error: %s",ex.what());
                return;
            }
        }
        else
        {
            transWrench.wrench = msg->wrench;
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
        _wrenchSub = _nh.subscribe("kms40", 1000, &WrenchTransformer::kms40Callback, this);
        _wrenchPub = _nh.advertise<geometry_msgs::WrenchStamped>("kms40_transformed", 1);

        if( !ros::param::get("~transformFrameId", transFrameId) )
        {
            ROS_ERROR("Cannot find transformFrameId @ parameterServer");
            _nh.shutdown();
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

            r.sleep();
        }

    }

};



int main( int argc, char** argv )
{
    ros::init(argc, argv, "WrenchTransformer");

    ROS_ERROR("Node not working as hoped. Math failed");
    //WrenchTransformer wrenchTransformer(ros::this_node::getName());

    return 0;
}
