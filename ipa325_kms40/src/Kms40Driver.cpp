#include <iostream>
#include <string>
#include <boost/asio.hpp>
#include <boost/algorithm/string.hpp>

#include "ros/ros.h"
#include "geometry_msgs/WrenchStamped.h"

using boost::asio::ip::tcp;

static bool transformKMS2WrenchMsg(const char* i_line, geometry_msgs::WrenchStamped& o_msg)
{
    try
    {
        if(boost::algorithm::starts_with(i_line, "F="))
        {
            int timestamp;
            sscanf(i_line, "F={%lf,%lf,%lf,%lf,%lf,%lf},%d",   &o_msg.wrench.force.x,
                                                               &o_msg.wrench.force.y,
                                                               &o_msg.wrench.force.z,
                                                               &o_msg.wrench.torque.x,
                                                               &o_msg.wrench.torque.y,
                                                               &o_msg.wrench.torque.z,
                                                               &timestamp);
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

    std::string ip, port;

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

    ros::NodeHandle n;
    ros::Publisher wrench_pub = n.advertise<geometry_msgs::WrenchStamped>("kms40", 1000);
    ros::Rate loop_rate(550); //500Hz + epsilon (ros::sleep may not be too precise)


    while (ros::ok())
    {
        ROS_INFO("Trying to connect to KMS40 ...");
        try
        {
            // set up connection to kms
            tcp::iostream s(ip, port);
            if (!s)
            {
                ROS_WARN("Connection failed, retry in 5 seconds...");
                ros::Duration(5, 0).sleep();
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
                /**
               * The publish() function is how you send messages. The parameter
               * is the message object. The type of this object must agree with the type
               * given as a template parameter to the advertise<>() call, as was done
               * in the constructor above.
               */
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

    ros::spinOnce();
    return 0;
}
