#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/WrenchStamped.h>

#include <boost/thread.hpp>

//struct wrenchVis
//{
//    double rx;
//    double ry;
//    double rz;
//    double rw;

//    double length;
//};

class WrenchMarkerPublisher
{
protected:
    ros::NodeHandle _nh;
    ros::Subscriber _wrenchSub;
    ros::Publisher _markerPub;

    visualization_msgs::Marker _markerMsg;

    boost::shared_mutex _wrenchDataAccess;
    double _wrenchData[6];
//    wrenchVis _forceVis;
//    wrenchVis _torqueVis;

    void kms40Callback(const geometry_msgs::WrenchStampedConstPtr& msg)
    {
        // get upgradable access
        boost::upgrade_lock<boost::shared_mutex> lock(_wrenchDataAccess);

        // get exclusive access
        boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);

        ROS_INFO("I heard force x: [%f]", msg->wrench.force.x);

        _wrenchData[0] = msg->wrench.force.x;
        _wrenchData[1] = msg->wrench.force.y;
        _wrenchData[2] = msg->wrench.force.z;
        _wrenchData[3] = msg->wrench.torque.x;
        _wrenchData[4] = msg->wrench.torque.y;
        _wrenchData[5] = msg->wrench.torque.z;
    }

    void getWrench(double(&wrench)[6])
    {
        boost::shared_lock<boost::shared_mutex> lock(_wrenchDataAccess);
        for (int i=0; i<6; i++)
        {
            wrench[i] = _wrenchData[i];
        }
    }


public:
    WrenchMarkerPublisher(std::string name)
    {
        memset(&_wrenchData, 0, sizeof(_wrenchData));

        _wrenchSub = _nh.subscribe("kms40", 1000, &WrenchMarkerPublisher::kms40Callback, this);
        _markerPub = _nh.advertise<visualization_msgs::Marker>("kms40_marker", 1);

        // initializing markerMsg
        _markerMsg.header.frame_id = "/kms40";
        _markerMsg.id = 0;

        // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
        _markerMsg.type = visualization_msgs::Marker::ARROW;

        // Set the marker action.  Options are ADD and DELETE
        _markerMsg.action = visualization_msgs::Marker::ADD;

        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        _markerMsg.pose.position.x = 0;
        _markerMsg.pose.position.y = 0;
        _markerMsg.pose.position.z = 0;

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        _markerMsg.scale.x = 1.0;
        _markerMsg.scale.y = 0.2;
        _markerMsg.scale.z = 0.2;

        // Set the color -- be sure to set alpha to something non-zero!
        _markerMsg.color.r = 0.0f;
        _markerMsg.color.g = 1.0f;
        _markerMsg.color.b = 0.0f;
        _markerMsg.color.a = 1.0;

        _markerMsg.lifetime = ros::Duration();

        ros::Rate r(50);

        double wrench[6] = {0};
        while (ros::ok())
        {

            // Set the scale of the marker -- 1x1x1 here means 1m on a side
            getWrench(wrench);

            _markerMsg.pose.orientation.x = wrench[0];
            _markerMsg.pose.orientation.y = wrench[1];
            _markerMsg.pose.orientation.z = wrench[2];
            _markerMsg.pose.orientation.w = 1.0;

            // Transform for vis
            double length = std::sqrt(  std::pow(wrench[0],2) +
                                        std::pow(wrench[1],2) +
                                        std::pow(wrench[2],2));
            _markerMsg.scale.x = length;

//            // Set the color -- be sure to set alpha to something non-zero!
//            marker.color.r = 0.0f;
//            marker.color.g = 1.0f;
//            marker.color.b = 0.0f;
//            marker.color.a = 1.0;

            // Publish the marker
            _markerPub.publish(_markerMsg);


            ros::spinOnce();

            r.sleep();
        }
    }

};



int main( int argc, char** argv )
{
    ros::init(argc, argv, "MarkerPublisher");

    WrenchMarkerPublisher markerPub(ros::this_node::getName());
    ros::spin();

    return 0;
    /*
  ros::init(argc, argv, "MarkerPublisher");
  ros::NodeHandle n;
  ros::Rate r(1);

  ros::Subscriber wrenchSub = n.subscribe("kms40", 1000, kms40Callback);

  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("kms40_force", 1);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::ARROW;

  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/kms40";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "kms40";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD and DELETE
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    marker_pub.publish(marker);

    r.sleep();
  }
  */
}
