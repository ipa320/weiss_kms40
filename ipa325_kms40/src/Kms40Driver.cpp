#include <iostream>
#include <string>
#include <boost/asio.hpp>
#include <boost/algorithm/string.hpp>

#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/WrenchStamped.h>

using boost::asio::ip::tcp;

static bool transformKMS2WrenchMsg(const char* i_line, geometry_msgs::WrenchStamped& o_msg)
{
    try
    {
        if(boost::algorithm::starts_with(i_line, "F="))
        {
            //TODO: translate received timestamp in rostime format and include into msg
            int timestamp;

            sscanf(i_line, "F={%lf,%lf,%lf,%lf,%lf,%lf},%d",
                   &o_msg.wrench.force.x,
                   &o_msg.wrench.force.y,
                   &o_msg.wrench.force.z,
                   &o_msg.wrench.torque.x,
                   &o_msg.wrench.torque.y,
                   &o_msg.wrench.torque.z,
                   &timestamp);

            o_msg.header.stamp = ros::Time::now();
            o_msg.header.frame_id = "/kms40";
        }
        else
        {
            ROS_WARN("Unrecognizable token: %s", i_line);
        }
    }
    catch (std::exception& e)
    {
        ROS_WARN("Error (%s) while parsing: %s", e.what(), i_line);
        return false;
    }

    return true;
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "kms40");

    std::string ip, port, topic;
    bool isDummy;
    double dummyValues[6] = {0};

    std::string parentFrame;

    if( !ros::param::get("~IP_address", ip) )
    {
        ROS_WARN("Cannot find IP_address @ paramServer, using default (192.168.1.30)");
        ip = "192.168.1.30";
    }

    if( !ros::param::get("~port", port) )
    {
        ROS_WARN("Cannot find port @ paramServer, using default (1000)");
        port = "1000";
    }

    if( !ros::param::get("~dummyMode", isDummy) )
    {
        ROS_WARN("Cannot find dummyMode @ paramServer, using default (true)");
        isDummy = true;
    }

    if(isDummy)
    {
        XmlRpc::XmlRpcValue dummyValuesXmlRpc;
        if( !ros::param::get("~dummyValues", dummyValuesXmlRpc) )
        {
            ROS_WARN("Cannot find dummyValues @ paramServer, using default ([0, 0, 0, 0, 0, 0])");
        }
        else
        {
            for (int i = 0; (i < dummyValuesXmlRpc.size()) && (i < 6); i++) {
                dummyValues[i] = (double) dummyValuesXmlRpc[i];
            }
        }
    }

    if( !ros::param::get("~parentFrame", parentFrame) )
    {
        ROS_WARN("Cannot find parentframe @ parameterServer, using default (world)");
        parentFrame = "world";
    }
    
    if( !ros::param::get("~topic", topic) )
    {
        ROS_WARN("Cannot find topic @ parameterServer, using default (/kms40)");
        topic = "kms40";
    }

    ros::NodeHandle n;
    ros::Publisher wrench_pub = n.advertise<geometry_msgs::WrenchStamped>(topic, 1000);
    ros::Rate loop_rate(550); //500Hz + epsilon (ros::sleep may not be too precise)

    while (ros::ok())
    {
        if (isDummy)
        {
            ROS_INFO_ONCE("DummyMode is active, publishing static test values ...");

            geometry_msgs::WrenchStamped msg;
            while(ros::ok())
            {
                msg.header.stamp = ros::Time::now();
                msg.header.frame_id = parentFrame;

                msg.wrench.force.x = dummyValues[0];
                msg.wrench.force.y = dummyValues[1];
                msg.wrench.force.z = dummyValues[2];
                msg.wrench.torque.x = dummyValues[3];
                msg.wrench.torque.y = dummyValues[4];
                msg.wrench.torque.z = dummyValues[5];

                //tfBroadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), parentFrame, "kms40"));
                wrench_pub.publish(msg);

                ros::spinOnce();
                loop_rate.sleep();
            }
        }
        else //harvest data from sensor
        {
            ROS_INFO("Trying to connect to KMS40 ...");
            try
            {
                // set up connection to kms
                tcp::iostream s(ip, port);
                if (!s)
                {
                    ROS_WARN("Connection failed, retry in 5 seconds...");
                    ros::Duration(1, 0).sleep();
                    continue;
                }
                ROS_INFO("Connection established");

                std::string inputLine;

                // Set kms in publishing mode
                ROS_INFO("Starting data acquisition");
                s << "L1()" << std::endl;

                std::getline(s, inputLine);
                if (boost::algorithm::lexicographical_compare(inputLine, "L1"))
                {
                    ROS_WARN("Data acquisition mode was not replied by KMS! Trying to reconnect.");
                    continue;
                }

                // Get ready for harvesting data
                geometry_msgs::WrenchStamped msg;
                while(ros::ok())
                {
                    std::getline(s, inputLine);
                    msg.header.stamp = ros::Time::now();

                    // if data harvest fails continue to loop without publishing
                    if (!transformKMS2WrenchMsg(inputLine.c_str(), msg))
                    {
                        continue;
                    }

                    msg.header.frame_id = parentFrame;

                    wrench_pub.publish(msg);

                    ros::spinOnce();
                    loop_rate.sleep();
                }

                // Tearing down data acquisition
                s << "L0()" << std::endl;
                ROS_INFO("Command for shutting down KMS40 has been send.");
            }
            catch (std::exception& e)
            {
                ROS_ERROR("Exception: %s", e.what());
            }
        }
    }

    ros::spinOnce();
    return 0;
}
