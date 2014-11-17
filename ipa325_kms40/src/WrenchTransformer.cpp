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

    std::string _transFrameId;
    std::string _subTopic;
    std::string _pubTopic;
    tf::TransformListener _listener;
    double _offset[6];

    void kms40Callback(const geometry_msgs::WrenchStampedConstPtr& data)
    {
        geometry_msgs::WrenchStamped transWrench;
        transWrench.header.stamp = data->header.stamp;
        transWrench.header.frame_id = _transFrameId;

        if (data->header.frame_id.compare(_transFrameId) != 0)
        {

            tf::Matrix3x3 transRotMatrix, M_transMatrix;

            tf::Vector3 transTranslation, F_measured, M_measured , F_target , M_target;

            F_measured[0] = data->wrench.force.x;
            F_measured[1] = data->wrench.force.y;
            F_measured[2] = data->wrench.force.z;

            M_measured[0] = data->wrench.torque.x;
            M_measured[1] = data->wrench.torque.y;
            M_measured[2] = data->wrench.torque.z;


            tf::StampedTransform transform;
            // Listen to kms40 to the tool frame
            try{
//                _listener.waitForTransform(data->header.frame_id, _transFrameId, ros::Time(0), ros::Duration(0.1));
                _listener.lookupTransform(_transFrameId, data->header.frame_id, ros::Time(0), transform);
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("%s",ex.what());
                return;
            }

            // Finding the rotation Matrix from kms40 to tool_tcp
            transRotMatrix = transform.getBasis();

            //Finding the translation vector from kms40 to tool_tcp
            transTranslation = transform.getOrigin();

            // Calculating the Force in the new tool_tcp
            F_target = transRotMatrix * F_measured;


            // Define the Omega matrix
            M_transMatrix[0][0] = 0;
            M_transMatrix[0][1] = -transTranslation[2];
            M_transMatrix[0][2] = transTranslation[1];
            M_transMatrix[1][0] = transTranslation[2];
            M_transMatrix[1][1] = 0;
            M_transMatrix[1][2] = -transTranslation[0];
            M_transMatrix[2][0] = -transTranslation[1];
            M_transMatrix[2][1] = transTranslation[0];
            M_transMatrix[2][2] = 0;

            //Calculating the moment at the tool_tcp

            M_target = (M_transMatrix * transRotMatrix) * F_measured + transRotMatrix * M_measured;

            transWrench.wrench.force.x = F_target[0];
            transWrench.wrench.force.y = F_target[1];
            transWrench.wrench.force.z = F_target[2];
            transWrench.wrench.torque.x = M_target[0];
            transWrench.wrench.torque.y = M_target[1];
            transWrench.wrench.torque.z = M_target[2];

        }
        else
        {
            transWrench.wrench = data->wrench;
        }

        // get upgradable access
        boost::upgrade_lock<boost::shared_mutex> lock(_wrenchDataAccess);

        // get exclusive access
        boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);
        
        // lth todo
        transWrench.wrench.force.x -= _offset[0];
        transWrench.wrench.force.y -= _offset[1];
        transWrench.wrench.force.z -= _offset[2];
        transWrench.wrench.torque.x -= _offset[3];
        transWrench.wrench.torque.y -= _offset[4];
        transWrench.wrench.torque.z -= _offset[5];

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

        if( !ros::param::get("~transformFrameId", _transFrameId) )
        {
            ROS_ERROR("Cannot find transformFrameId @ parameterServer");
            return;
        }

        if( !ros::param::get("~subscribeTopic", _subTopic) )
        {
            ROS_ERROR("Cannot find subscribeTopic @ parameterServer");
            return;
        }

        if( !ros::param::get("~publishTopic", _pubTopic) )
        {
            ROS_ERROR("Cannot find publishTopic @ parameterServer");
            return;
        }

        double frequency = 500.0;
        if( !ros::param::get("~publishingRate", frequency) )
        {
            ROS_ERROR("Cannot find publishingRate @ parameterServer, using default 500Hz");
        }

        // LTH TODO: Get from (dynamic) parameter
        if( !ros::param::get("~offset/Fx", _offset[0]) ) { _offset[0]= 0.0f; };
        if( !ros::param::get("~offset/Fy", _offset[1]) ) { _offset[1]= 0.0f; };
        if( !ros::param::get("~offset/Fz", _offset[2]) ) { _offset[2]= 0.0f; };
        if( !ros::param::get("~offset/Mx", _offset[3]) ) { _offset[3]= 0.0f; };
        if( !ros::param::get("~offset/My", _offset[4]) ) { _offset[4]= 0.0f; };
        if( !ros::param::get("~offset/Mz", _offset[5]) ) { _offset[5]= 0.0f; };
        //_offset = 3.0f;

        ros::Rate r(frequency);

        _wrenchPub = _nh.advertise<geometry_msgs::WrenchStamped>(_pubTopic, 1000);
        _wrenchSub = _nh.subscribe(_subTopic, 1000, &WrenchTransformer::kms40Callback, this);

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

    //    ROS_ERROR("Node not working as hoped. Math failed");
    WrenchTransformer wrenchTransformer(ros::this_node::getName());

    return 0;
}
