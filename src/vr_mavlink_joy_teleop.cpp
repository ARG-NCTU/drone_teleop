#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/SetMode.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>

#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>

using namespace std;

class Teleop
{
  private:
    // ROS handles
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber sub_joy_;
    ros::Publisher pub_twist_;
    ros::Publisher pub_track_trigger_;

    ros::ServiceClient srv_arming_;
    ros::ServiceClient srv_land_;
    ros::ServiceClient srv_offboard_;
    ros::ServiceClient srv_param_set_;
    ros::ServiceClient srv_param_get_;

    ros::Timer timer_pub_;

    // ROS messages
    geometry_msgs::Twist twist_;
    std_msgs::Bool trigger_;

    // ROS params
    std::unordered_map<std::string, std::unordered_map<std::string, double>> throttle_;
    double control_rate_;

    // Private variables
    bool enable_;
    std::string speed_mode_;
    std::string mode_;

    std::vector<double> axes_;
    std::vector<double> buttons_;
    std::unordered_map<std::string, int> axes_map_;
    std::unordered_map<std::string, int> button_map_;
    std::unordered_map<std::string, std::string> function_map_;

    void getParam();
    void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
    void timerPubCallback(const ros::TimerEvent& event);
    void initMaps();
    bool isPressed(std::string function, const sensor_msgs::Joy::ConstPtr& msg);
    bool isTrigger(std::string function, const sensor_msgs::Joy::ConstPtr& msg);

  public:
    Teleop(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
};

Teleop::Teleop(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
  : nh_(nh), nh_private_(nh_private), enable_(true), speed_mode_(std::string("high")), mode_(std::string("HOLD"))
{
    getParam();

    sub_joy_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &Teleop::joyCallback, this);
    pub_twist_ = nh_private_.advertise<geometry_msgs::Twist>("twist", 10);
    pub_track_trigger_ = nh_.advertise<std_msgs::Bool>("track_trigger", 10);

    srv_arming_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    srv_land_ = nh_.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    srv_offboard_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    srv_param_set_ = nh_.serviceClient<mavros_msgs::ParamSet>("/mavros/param/set");
    srv_param_get_ = nh_.serviceClient<mavros_msgs::ParamGet>("/mavros/param/get");

    timer_pub_ = nh_.createTimer(ros::Duration(1.0 / control_rate_), &Teleop::timerPubCallback, this, false, false);

    axes_.resize(10);
    buttons_.resize(10);

    initMaps();

    timer_pub_.start();
    ROS_INFO_STREAM("Teleop node initialized in " << control_rate_ << " Hz");
}

void Teleop::getParam()
{
    nh_private_.param<double>("throttle_high_throttle", throttle_["high"]["throttle"], 2.0);
    nh_private_.param<double>("throttle_high_linear", throttle_["high"]["linear"], 2.0);
    nh_private_.param<double>("throttle_high_angular", throttle_["high"]["angular"], 1.0);

    nh_private_.param<double>("throttle_low_throttle", throttle_["low"]["throttle"], 0.5);
    nh_private_.param<double>("throttle_low_linear", throttle_["low"]["linear"], 0.5);
    nh_private_.param<double>("throttle_low_angular", throttle_["low"]["angular"], 0.5);

    nh_private_.param<double>("control_rate", control_rate_, 10.0);

    ROS_INFO_STREAM(ros::this_node::getName() << "throttle_high_throttle: " << throttle_["high"]["throttle"]);
    ROS_INFO_STREAM(ros::this_node::getName() << "throttle_high_linear: " << throttle_["high"]["linear"]);
    ROS_INFO_STREAM(ros::this_node::getName() << "throttle_high_angular: " << throttle_["high"]["angular"]);

    ROS_INFO_STREAM(ros::this_node::getName() << "throttle_low_throttle: " << throttle_["low"]["throttle"]);
    ROS_INFO_STREAM(ros::this_node::getName() << "throttle_low_linear: " << throttle_["low"]["linear"]);
    ROS_INFO_STREAM(ros::this_node::getName() << "throttle_low_angular: " << throttle_["low"]["angular"]);

    ROS_INFO_STREAM(ros::this_node::getName() << "control_rate: " << control_rate_);
    return;
}

void Teleop::joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
    bool select = msg->axes[axes_map_["SELECT"]] == 5;

    int modeAxis = msg->axes[axes_map_["MODE"]];
    if (modeAxis >= 0 && modeAxis <= 2)
    {
        const std::string modes[] = { "MANUAL", "AUTO", "HOLD" };
        mode_ = modes[modeAxis];
    }

    if (isTrigger("ARM", msg))
    {
        ROS_INFO_STREAM("ARM");
        mavros_msgs::CommandBool srv;
        srv.request.value = true;
        if (srv_arming_.exists())
        {
            auto res = srv_arming_.call(srv);
            if (res)
            {
                ROS_INFO_STREAM("Arming");
            }
            else
            {
                ROS_ERROR_STREAM("Arming failed");
            }
        }
        else
        {
            ROS_ERROR_STREAM("Arming service does not exist");
        }
    }

    if (isTrigger("OFFBOARD", msg))
    {
        ROS_INFO_STREAM("OFFBOARD");
        if (srv_offboard_.exists())
        {
            mavros_msgs::SetMode srv;
            srv.request.custom_mode = "OFFBOARD";
            auto res = srv_offboard_.call(srv);
            if (res)
            {
                ROS_INFO_STREAM("Offboard");
            }
            else
            {
                ROS_ERROR_STREAM("Offboard failed");
            }
        }
        else
        {
            ROS_ERROR_STREAM("Offboard service does not exist");
        }
    }

    if (isTrigger("LAND", msg))
    {
        ROS_INFO_STREAM("LAND");
        if (srv_land_.exists())
        {
            mavros_msgs::CommandTOL srv;
            srv.request.min_pitch = 0;
            srv.request.yaw = 0;
            srv.request.latitude = 0;
            srv.request.longitude = 0;
            srv.request.altitude = 0;
            auto res = srv_land_.call(srv);
            if (res)
            {
                ROS_INFO_STREAM("Land");
            }
            else
            {
                ROS_ERROR_STREAM("Land failed");
            }
        }
        else
        {
            ROS_ERROR_STREAM("Land service does not exist");
        }
    }

    if (isTrigger("INITIALMAVLINK", msg))
    {
        ROS_INFO_STREAM("INITIALMAVLINK");
        if (srv_param_set_.exists() && srv_param_get_.exists())
        {
            mavros_msgs::ParamSet srv_param_set;
            srv_param_set.request.param_id = "COM_RCL_EXCEPT";
            srv_param_set.request.value.integer = 4;
            srv_param_set_.call(srv_param_set);

            mavros_msgs::ParamGet srv_param_get;
            srv_param_get.request.param_id = "COM_RCL_EXCEPT";
            srv_param_get_.call(srv_param_get);
            ROS_INFO_STREAM("Set COM_RCL_EXCEPT: " << srv_param_set.request.value.integer
                                                   << ". Get COM_RCL_EXCEPT: " << srv_param_get.response.value.integer);

            srv_param_set.request.param_id = "COM_OBL_ACT";
            srv_param_set.request.value.integer = -1;
            srv_param_set_.call(srv_param_set);

            srv_param_get.request.param_id = "COM_OBL_ACT";
            srv_param_get_.call(srv_param_get);
            ROS_INFO_STREAM("Set COM_OBL_ACT: " << srv_param_set.request.value.integer
                                                << ". Get COM_OBL_ACT: " << srv_param_get.response.value.integer);
        }
        else
        {
            ROS_ERROR_STREAM("Param service does not exist");
        }
    }

    if (select)
    {
        if (mode_ == "MANUAL")
        {
            ROS_INFO_STREAM("MANUAL");
            trigger_.data = false;
            enable_ = true;
        }
        else if (mode_ == "AUTO")
        {
            ROS_INFO_STREAM("AUTO");
            enable_ = false;
            trigger_.data = true;
        }
        else if (mode_ == "HOLD")
        {
            trigger_.data = false;
            if (axes_[axes_map_["MODE"]] != 2)
            {
                enable_ = false;
                ROS_INFO_STREAM("HOLD");
                if (srv_offboard_.exists())
                {
                    mavros_msgs::SetMode srv;
                    srv.request.custom_mode = "AUTO.LOITER";
                    auto res = srv_offboard_.call(srv);
                    if (res)
                    {
                        ROS_INFO_STREAM("HOLD");
                    }
                    else
                    {
                        ROS_ERROR_STREAM("HOLD failed");
                    }
                }
                else
                {
                    ROS_ERROR_STREAM("HOLD service does not exist");
                }
            }
        }
    }
    std::copy(msg->axes.begin(), msg->axes.end(), axes_.begin());
    std::copy(msg->buttons.begin(), msg->buttons.end(), buttons_.begin());
    return;
}

void Teleop::timerPubCallback(const ros::TimerEvent& event)
{
    if (enable_)
    {
        twist_.linear.x = throttle_[speed_mode_]["linear"] * axes_[axes_map_["RY"]];
        twist_.linear.y = throttle_[speed_mode_]["linear"] * axes_[axes_map_["RX"]];
        twist_.linear.z = throttle_[speed_mode_]["throttle"] * axes_[axes_map_["LY"]];
        twist_.angular.z = throttle_[speed_mode_]["angular"] * axes_[axes_map_["LX"]];
        pub_twist_.publish(twist_);
        ROS_INFO_STREAM_THROTTLE(1.0, "drone: "
                                          << "RY: " << axes_[axes_map_["RY"]] << ", RX: " << axes_[axes_map_["RX"]]
                                          << ", LY: " << axes_[axes_map_["LY"]] << ", LX: " << axes_[axes_map_["LX"]]);
    }
    else
    {
    }
    pub_track_trigger_.publish(trigger_);
}

void Teleop::initMaps()
{
    function_map_["THROTTLE"] = std::string("LY");
    function_map_["YAW"] = std::string("LX");
    function_map_["X"] = std::string("RY");
    function_map_["Y"] = std::string("RX");

    function_map_["AUTO"] = std::string("AUTO");
    function_map_["MANUAL"] = std::string("MANUAL");
    function_map_["HOLD"] = std::string("HOLD");

    function_map_["INITIALMAVLINK"] = std::string("X");
    function_map_["ARM"] = std::string("Y");
    function_map_["OFFBOARD"] = std::string("A");
    function_map_["LAND"] = std::string("B");

    axes_map_.clear();
    axes_map_["SELECT"] = 1;
    axes_map_["MODE"] = 2;
    axes_map_["LY"] = 6;
    axes_map_["LX"] = 7;
    axes_map_["RY"] = 8;
    axes_map_["RX"] = 9;

    button_map_.clear();
    button_map_["MANUAL"] = 0;
    button_map_["AUTO"] = 1;
    button_map_["HOLD"] = 2;
    button_map_["X"] = 3;
    button_map_["Y"] = 4;
    button_map_["A"] = 5;
    button_map_["B"] = 6;
}

bool Teleop::isPressed(const std::string function, const sensor_msgs::Joy::ConstPtr& msg)
{
    // if founciton is found in function_map_
    if (function_map_.find(function) != function_map_.end())
    {
        return msg->buttons[button_map_[function_map_[function]]];
    }
    else
    {
        return msg->buttons[button_map_[function]];
    }
}

bool Teleop::isTrigger(const std::string function, const sensor_msgs::Joy::ConstPtr& msg)
{
    if (function_map_.find(function) != function_map_.end())
    {
        return isPressed(function, msg) && !buttons_[button_map_[function_map_[function]]];
    }
    else
    {
        return isPressed(function, msg) && !buttons_[button_map_[function]];
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "drone_teleop");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    Teleop teleop(nh, nh_private);
    ros::spin();
    return 0;
}
