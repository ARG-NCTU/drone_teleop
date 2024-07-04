#include <iostream>
#include <math.h> 

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandTOL.h>
#include <std_srvs/Trigger.h>
#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/ParamSet.h>
using namespace std; 

/**
 * @file drone_vr_teleop.cpp
 * @brief ROS node for controlling a drone using VR joystick inputs.
 * 
 * This code defines a Controller class to manage drone teleoperation via VR joystick inputs.
 * 
 * Functions:
 *  - Controller::Controller: Constructor that initializes the ROS node handles, subscribers, publishers, and service clients.
 *  - Controller::getParam: Retrieves parameters from the ROS parameter server.
 *  - Controller::joyCallback: Callback function for joystick messages. Processes the joystick input and triggers corresponding actions.
 *  - Controller::toggleOutput: Toggles the joystick output state.
 *  - Controller::srvArming: Arms the drone.
 *  - Controller::srvOffboard: Sets the drone to offboard mode.
 *  - Controller::srvLand: Lands the drone.
 *  - Controller::srvHover: Sets the drone to hover mode.
 *  - Controller::srvFlypose: Sends a trigger to fly to the initial pose.
 *  - Controller::control: Controls the drone based on the joystick inputs.
 * 
 * ROS Subscribers:
 *  - sub_joy: Subscribes to "vr_joy_drone" topic to receive joystick inputs.
 * 
 * ROS Publishers:
 *  - pub_twist: Publishes twist messages to the "drone_twist" topic to control the drone's movement.
 *  - pub_speed_state: Publishes boolean messages to the "speed_state" topic to indicate high/low speed state.
 * 
 * ROS Service Clients:
 *  - srv_arming: Client for "mavros/cmd/arming" service to arm the drone.
 *  - srv_offboard: Client for "mavros/set_mode" service to set the drone mode to offboard.
 *  - srv_land: Client for "mavros/cmd/land" service to land the drone.
 *  - srv_hover: Client for "mavros/set_mode" service to set the drone mode to hover.
 *  - srv_flypose: Client for "/flypose" service to send a trigger to fly to the initial pose.
 *  - srv_param_set_: Client for "/mavros/param/set" service to set parameters.
 *  - srv_param_get_: Client for "/mavros/param/get" service to get parameters.
 */

class Controller{
    private:
        ros::NodeHandle n;
        ros::NodeHandle nh_private;
        ros::Subscriber sub_joy;
        ros::Publisher pub_twist;
        ros::Publisher pub_speed_state;
        ros::Publisher pub_enable_auto_mode_success_; //publish bt condition node of auto switch (by operator)

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
        bool enable_auto_mode_success_ = false;
        bool mode_switch = false;

        bool high_speed = false;

        // {vr trigger + left_hand mushroom back/forth (For up/down), vr trigger + left_hand mushroom left/right (For rotate), vr trigger + left_hand mushroom back/forth (For move back and forth), vr gripper + left_hand mushroom left/right (For move left and right)}
        float m_axes[4] = {0.0, 0.0, 0.0, 0.0};
        // {VR X(set_param & Arming & Offboard & flypose service), Y(Hover), A (for gripper), B (mannual/auto), left_grip_counter for high low speed } 
        int m_buttons[5] = {0, 0, 0, 0, 0};  

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

Controller :: Controller(): nh_private("~"){
    sub_joy = n.subscribe<sensor_msgs::Joy>("vr_joy_drone", 1,  &Controller::joyCallback, this);
    pub_twist = n.advertise<geometry_msgs::Twist>("drone_twist", 10);
    pub_speed_state = n.advertise<std_msgs::Bool>("speed_state", 10);//True for high speed, False for low speed
    pub_enable_auto_mode_success_ = n.advertise<std_msgs::Bool>("/enable_auto_mode_success", 10);// True for auto mode enabled, False for auto mode disabled
    

    srv_arming = n.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    srv_offboard = n.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    srv_land = n.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
    srv_hover = n.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    srv_flypose = n.serviceClient<std_srvs::Trigger>("/flypose");
    srv_param_set_ = n.serviceClient<mavros_msgs::ParamSet>("/mavros/param/set");
    srv_param_get_ = n.serviceClient<mavros_msgs::ParamGet>("/mavros/param/get");
}

void Controller :: getParam(){
    nh_private.getParam("highspeed_linear",  m_highspeed_linear);
    nh_private.getParam("highspeed_throttle", m_highspeed_throttle);
    nh_private.getParam("highspeed_angular", m_highspeed_angular);

    nh_private.getParam("lowspeed_linear",  m_lowspeed_linear);
    nh_private.getParam("lowspeed_throttle", m_lowspeed_throttle);
    nh_private.getParam("lowspeed_angular", m_lowspeed_angular);
    nh_private.getParam("mode_switch", mode_switch);
    return;
}


void Controller :: joyCallback(const sensor_msgs::Joy::ConstPtr& msg){
    for(int i=0; i<4; i++){ m_axes[i] = msg->axes[i]; }
    for(int i=0; i<5; i++){ m_buttons[i] = msg->buttons[i]; }

    // mode & arming
    bool state_arm = 0;
    bool state_offboard = 0;
    //bool state_land = 0; //no valid button for land
    bool state_hover = 0;
    bool flypose_response = 0;

    // Check if all joy data is zero
    bool allZero = true;
    for(int i = 0; i < 4; i++){
        if(m_axes[i] != 0.0){
            allZero = false;
            break;
        }
    }
    allZero = false;
    // If all data is zero, switch to position mode, otherwise switch to offboard mode
    // if(mode_switch && allZero){
    //     ROS_INFO("Switching to Position Mode");
    //     mavros_msgs::SetMode positionModeSrv;
    //     positionModeSrv.request.custom_mode = "POSCTL"; 
    //     if(srv_hover.call(positionModeSrv)){
    //         ROS_INFO("Position mode set successfully");
    //     }else{
    //         ROS_ERROR("Failed to set position mode");
    //     }
    // }else{
    //     ROS_INFO("Switching to Offboard Mode");
    //     if(!srvOffboard()){
    //         ROS_ERROR("Failed to set offboard mode");
    //     }
    // }

    
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
        enable_auto_mode_success_ = true; 
        
    }
    else if(m_buttons[3] == 0){
        enable_auto_mode_success_ = false;
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
        if(m_buttons[4] == 0){
            throttle = m_lowspeed_throttle;
            linear  = m_lowspeed_linear;
            angular = m_lowspeed_angular;
            high_speed = false;
        }else{
            throttle = m_highspeed_throttle;
            linear  = m_highspeed_linear;
            angular = m_highspeed_angular;
            high_speed = true;
        }
        

        pub_msg_twist.linear.z = m_axes[0] * throttle;
        pub_msg_twist.angular.z = m_axes[1] * angular;
        pub_msg_twist.linear.x = m_axes[2] * linear;
        pub_msg_twist.linear.y = m_axes[3] * linear;
        //ROS_INFO("Speed: %f, %f, %f, %f", pub_msg_twist.linear.x, pub_msg_twist.linear.y, pub_msg_twist.linear.z, pub_msg_twist.angular.z);
        pub_twist.publish(pub_msg_twist);
        std_msgs::Bool speed_state;
        speed_state.data = high_speed;//true for high speed, false for low speed
        pub_speed_state.publish(speed_state);
        std_msgs::Bool enable_auto_mode;
        enable_auto_mode.data = enable_auto_mode_success_;
        pub_enable_auto_mode_success_.publish(enable_auto_mode);

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