#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <mavros_msgs/CommandLong.h>

// Service callback for drone kill
bool kill_drone_callback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");

    mavros_msgs::CommandLong srv;
    srv.request.broadcast = false;
    srv.request.command = 400;
    srv.request.confirmation = 0;
    srv.request.param1 = 0.0;
    srv.request.param2 = 21196;
    srv.request.param3 = 0.0;
    srv.request.param4 = 0.0;
    srv.request.param5 = 0.0;
    srv.request.param6 = 0.0;
    srv.request.param7 = 0.0;

    if (client.call(srv))
    {
        ROS_INFO("Command sent successfully");
        res.success = true;
        res.message = "Command executed successfully.";
    }
    else
    {
        ROS_ERROR("Failed to send command");
        res.success = false;
        res.message = "Failed to execute command.";
    }
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "drone_kill_node");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("/drone/kill", kill_drone_callback);
    ROS_INFO("Drone Kill Service Ready.");
    
    ros::spin();

    return 0;
}
