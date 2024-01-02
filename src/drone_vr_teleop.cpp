#include <iostream>
#include <math.h> 

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandTOL.h>
#include <std_srvs/Trigger.h>
#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/ParamSet.h>
using namespace std; 


class Controller{
    private:
        ros::NodeHandle n;
        ros::Subscriber sub_joy;
        ros::Publisher pub_twist;

        ros::ServiceClient srv_arming;
        ros::ServiceClient srv_offboard;
        ros::ServiceClient srv_land;

        ros::ServiceClient srv_hover;
        ros::ServiceClient srv_flypose;
        ros::ServiceClient srv_param_set_;
        ros::ServiceClient srv_param_get_;

        float m_highspeed_linear = 3.0;
        float m_highspeed_throttle = 2.0;
        float m_highspeed_angular = 1;

        float m_lowspeed_linear = 1.0;
        float m_lowspeed_throttle = 1.0;
        float m_lowspeed_angular = 0.5;

        bool enable_joy_output = true;

        // {vr trigger + left_hand mushroom back/forth (For up/down), vr trigger + left_hand mushroom left/right (For rotate), vr trigger + left_hand mushroom back/forth (For move back and forth), vr gripper + left_hand mushroom left/right (For move left and right)}
        float m_axes[4] = {0.0, 0.0, 0.0, 0.0};
        // {VR X(set_param & Arming & Offboard & flypose service), Y(Hover), A (for gripper), B (mannual/auto) } 
        int m_buttons[4] = {0, 0, 0, 0};  

    public:
        Controller();
        void getParam();
        void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
        void toggleOutput();
        bool srvArming();
        bool srvOffboard();
        bool srvLand();
        bool srvHover();
        bool srvFlypose();
        bool srvGripper(bool open);
        void control();
};

Controller :: Controller(){
    sub_joy = n.subscribe<sensor_msgs::Joy>("vr_joy_drone", 1,  &Controller::joyCallback, this);
    pub_twist = n.advertise<geometry_msgs::Twist>("drone_twist", 10);
    

    srv_arming = n.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    srv_offboard = n.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    srv_land = n.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
    srv_hover = n.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    srv_flypose = n.serviceClient<std_srvs::Trigger>("/flypose");
    srv_param_set_ = n.serviceClient<mavros_msgs::ParamSet>("/mavros/param/set");
    srv_param_get_ = n.serviceClient<mavros_msgs::ParamGet>("/mavros/param/get");
}

void Controller :: getParam(){
    n.getParam("/drone_teleop/highspeed_linear",  m_highspeed_linear);
    n.getParam("/drone_teleop/highspeed_throttle", m_highspeed_throttle);
    n.getParam("/drone_teleop/highspeed_angular", m_highspeed_angular);

    n.getParam("/drone_teleop/lowspeed_linear",  m_lowspeed_linear);
    n.getParam("/drone_teleop/lowspeed_throttle", m_lowspeed_throttle);
    n.getParam("/drone_teleop/lowspeed_angular", m_lowspeed_angular);
    return;
}


void Controller :: joyCallback(const sensor_msgs::Joy::ConstPtr& msg){
    for(int i=0; i<4; i++){ m_axes[i] = msg->axes[i]; }
    for(int i=0; i<4; i++){ m_buttons[i] = msg->buttons[i]; }

    // mode & arming
    bool state_arm = 0;
    bool state_offboard = 0;
    //bool state_land = 0; //no valid button for land
    bool state_hover = 0;
    bool flypose_response = 0;
    
    if(m_buttons[0] == 1){
        //VR left hand button X
        //set_param
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
        while(!state_arm && !state_offboard){
            //arming
            state_arm = srvArming();
            //offboard 
            state_offboard = srvOffboard();
        }

    }

    if(m_buttons[1] == 1){
        //VR left hand button Y
        while(!state_hover){ state_hover = srvHover(); }
        // fly initial pose
        flypose_response = srvFlypose();
    }

    if(m_buttons[3] == 1){
        //VR right hand button B push count
        enable_joy_output = false; 
        
    }
    else if(m_buttons[3] == 0){
        enable_joy_output = true;
    }

    return;
}

bool Controller :: srvArming(){
    ROS_INFO("Srv arming");
    mavros_msgs::CommandBool srv_msg;
    srv_msg.request.value = true;
    if(srv_arming.call(srv_msg)){ return true; }
    else{ return false; }
}

bool Controller :: srvOffboard(){
    ROS_INFO("Srv offboard");
    geometry_msgs::Twist pub_msg_twist;
    
    mavros_msgs::SetMode srv_msg;
    srv_msg.request.custom_mode = "OFFBOARD";
    if(srv_offboard.call(srv_msg)){ return true; }
    else{ return false; }
}

bool Controller :: srvLand(){
    ROS_INFO("Srv Land");

    mavros_msgs::CommandTOL srv_msg;
    srv_msg.request.min_pitch = 0;
    srv_msg.request.yaw = 0;
    srv_msg.request.latitude = 0;
    srv_msg.request.longitude = 0;
    srv_msg.request.altitude = 0;
    if(srv_land.call(srv_msg)){ return true; }
    else{ return false; }
}

bool Controller :: srvHover(){
    ROS_INFO("Srv Hover");
    
    mavros_msgs::SetMode srv_msg;
    srv_msg.request.custom_mode = "AUTO.LOITER";
    if(srv_hover.call(srv_msg)){ return true; }
    else{ return false; }
}

bool Controller :: srvFlypose(){
    ROS_INFO("Srv Flypose");
    std_srvs::Trigger srv_msg;
    if(srv_flypose.call(srv_msg)){ return true; }
    else{ return false; }

}

void Controller :: toggleOutput(){
    enable_joy_output = !enable_joy_output;
    ROS_INFO("Joystick enable: %s", enable_joy_output ? "true" : "false");
    return;
}


void Controller :: control(){
    ros::Rate rate(30);
    if(enable_joy_output){
        // twist
        geometry_msgs::Twist pub_msg_twist;
        // {rotate, up/down, left/right shift, forward/backward shift, 0, 0}

        float throttle, linear, angular;

        throttle = m_highspeed_throttle;
        linear  = m_highspeed_linear;
        angular = m_highspeed_angular;

        pub_msg_twist.linear.z = m_axes[0] * throttle;
        pub_msg_twist.angular.z = m_axes[1] * angular;
        pub_msg_twist.linear.x = m_axes[2] * linear;
        pub_msg_twist.linear.y = m_axes[3] * linear;
        //ROS_INFO("Speed: %f, %f, %f, %f", pub_msg_twist.linear.x, pub_msg_twist.linear.y, pub_msg_twist.linear.z, pub_msg_twist.angular.z);
        pub_twist.publish(pub_msg_twist);
            
    }
    rate.sleep();
    return;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "drone_vr_teleop");
    Controller joy;
    joy.getParam();
    while(ros::ok()){
        joy.control();
        ros::spinOnce();
    }
    return 0;
}