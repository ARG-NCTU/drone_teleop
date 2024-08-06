/// Drone F710 joy control need switch to D mode!!!
#include <iostream>
#include <math.h> 

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Trigger.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/ParamSet.h>

using namespace std; 

class Controller{
    private:
        ros::NodeHandle n;
        ros::Subscriber sub_joy;
        ros::Publisher pub_twist;
        ros::Publisher pub_auto;

        ros::ServiceClient srv_arming;
        ros::ServiceClient srv_offboard;
        ros::ServiceClient srv_land;
        ros::ServiceClient srv_hover;
        ros::ServiceClient srv_param_set_;
        ros::ServiceClient srv_param_get_;

        //for gripper control
        ros::ServiceClient srv_gripper_open;
        ros::ServiceClient srv_gripper_close;
        // for arm control
        ros::ServiceClient srv_grasp_pose;
        ros::ServiceClient srv_initial_pose;
        

        float m_highspeed_linear = 3.0;
        float m_highspeed_throttle = 2.0;
        float m_highspeed_angular = 1;

        float m_lowspeed_linear = 1.0;
        float m_lowspeed_throttle = 1.0;
        float m_lowspeed_angular = 0.5;

        bool enable_joy_output = true;
        bool enable_auto_mode = false;

        // {rotate, up/down, left/right shift, forward/backward shift, 0, 0}
        float m_axes[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        // {X, A(Arming), B(Offboard), Y(Land), LB, RB, LT, RT, back, start, Button stick left, Button stick right} 
        int m_buttons[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};  

    public:
        Controller();
        void getParam();
        void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
        void toggleOutput();
        bool srvArming();
        bool srvOffboard();
        bool srvLand();
        bool srvHover();
        void control();
};

Controller :: Controller(){
    sub_joy = n.subscribe<sensor_msgs::Joy>("joy", 1,  &Controller::joyCallback, this);
    pub_twist = n.advertise<geometry_msgs::Twist>("drone_twist", 10);
    pub_auto = n.advertise<std_msgs::Bool>("enable_auto_mode_success",10); 

    srv_arming = n.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    srv_offboard = n.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    srv_land = n.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
    srv_hover = n.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    srv_param_set_ = n.serviceClient<mavros_msgs::ParamSet>("/mavros/param/set");
    srv_param_get_ = n.serviceClient<mavros_msgs::ParamGet>("/mavros/param/get");
    
    //for gripper
    srv_gripper_open = n.serviceClient<std_srvs::Trigger>("gripper_open");
    srv_gripper_close = n.serviceClient<std_srvs::Trigger>("gripper_close");
    //for arm
    srv_grasp_pose = n.serviceClient<std_srvs::Trigger>("grasp_pose");
    srv_initial_pose = n.serviceClient<std_srvs::Trigger>("initial");

}

void Controller :: getParam(){
    n.getParam("/aerial_manipulation_teleop/highspeed_linear",  m_highspeed_linear);
    n.getParam("/aerial_manipulation_teleop/highspeed_throttle", m_highspeed_throttle);
    n.getParam("/aerial_manipulation_teleop/highspeed_angular", m_highspeed_angular);

    n.getParam("/aerial_manipulation_teleop/lowspeed_linear",  m_lowspeed_linear);
    n.getParam("/aerial_manipulation_teleop/lowspeed_throttle", m_lowspeed_throttle);
    n.getParam("/aerial_manipulation_teleop/lowspeed_angular", m_lowspeed_angular);
    return;
}


void Controller :: joyCallback(const sensor_msgs::Joy::ConstPtr& msg){
    for(int i=0; i<6; i++){ m_axes[i] = msg->axes[i]; }
    for(int i=0; i<12; i++){ m_buttons[i] = msg->buttons[i]; }

    // mode & arming
    bool state = 0;
    if(m_buttons[0] == 1){ // joy x button
        while(!state){ state = srvHover(); }
    }
    if(m_buttons[1] == 1){ // joy A button
        //add set param
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
        while(!state){ state = srvArming(); }
    }
    state = 0;
    if(m_buttons[2] == 1){ // joy B button
        while(!state){ state = srvOffboard(); }
    }
    state = 0;
    if(m_buttons[3] == 1){ // joy Y button
        while(!state){ state = srvLand(); }
    }

    if(m_buttons[9] == 1){ // joy start button
        enable_auto_mode = true;
        ROS_INFO("auto enable: %s", enable_auto_mode ? "true" : "false");
    }

    if(m_buttons[8] == 1){ // joy back button
        enable_auto_mode = false;
        ROS_INFO("auto enable: %s", enable_auto_mode ? "true" : "false");
    }

    if(m_axes[5] == 1){ // joy arrow up
        std_srvs::Trigger srv_msg;
        srv_gripper_open.call(srv_msg);
    }
    if(m_axes[5] == -1){ // joy arrow down
        std_srvs::Trigger srv_msg;
        srv_gripper_close.call(srv_msg);
    }
    if(m_buttons[5] == 1){ //RB for grasp pose service  
        std_srvs::Trigger srv_msg;
        srv_grasp_pose.call(srv_msg);
    }
    if(m_buttons[7] == 1){ //RT for initial pose service 
        std_srvs::Trigger srv_msg;
        srv_initial_pose.call(srv_msg);
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
    mavros_msgs::SetMode srv_msg;
    srv_msg.request.custom_mode = "OFFBOARD";
    if(srv_offboard.call(srv_msg)){ return true; }
    else{ return false; }
}

bool Controller :: srvHover(){
    ROS_INFO("Srv Hover");
    mavros_msgs::SetMode srv_msg;
    srv_msg.request.custom_mode = "AUTO.LOITER";
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

        if(m_buttons[6] == 1){ // joy LT button
            throttle = m_lowspeed_throttle;
            linear  = m_lowspeed_linear;
            angular = m_lowspeed_angular; 
        }else{
            throttle = m_highspeed_throttle;
            linear  = m_highspeed_linear;
            angular = m_highspeed_angular;
        }
        if(m_axes[0] != 0 || m_axes[1] != 0 || m_axes[2] != 0 || m_axes[3] != 0 ){ //joy LB button
            pub_msg_twist.angular.z = angular * m_axes[0];
            pub_msg_twist.linear.z = throttle * m_axes[1];

            pub_msg_twist.linear.y = linear * m_axes[2];
            pub_msg_twist.linear.x = linear * m_axes[3];
        }
        else{
            pub_msg_twist.angular.z = 0.0;
            pub_msg_twist.linear.z = 0.0;
            pub_msg_twist.linear.y = 0.0;
            pub_msg_twist.linear.x = 0.0;
        }
        pub_twist.publish(pub_msg_twist);
        
        std_msgs::Bool pub_msg_bool;
        pub_msg_bool.data = enable_auto_mode;
        pub_auto.publish(pub_msg_bool);
    }
    rate.sleep();
    return;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "drone_teleop");
    Controller joy;
    joy.getParam();
    while(ros::ok()){
        joy.control();
        ros::spinOnce();
    }
    return 0;
}