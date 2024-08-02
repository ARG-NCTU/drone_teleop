/// Drone F710 joy control need switch to D mode!!!
#include <iostream>
#include <math.h> 

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>
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

        ros::ServiceClient srv_arming;
        ros::ServiceClient srv_Guided;
        ros::ServiceClient srv_land;
        ros::ServiceClient srv_hover;
        ros::ServiceClient srv_param_set_;
        ros::ServiceClient srv_param_get_;

        float m_highspeed_linear = 3.0;
        float m_highspeed_throttle = 2.0;
        float m_highspeed_angular = 1;

        float m_lowspeed_linear = 1.0;
        float m_lowspeed_throttle = 1.0;
        float m_lowspeed_angular = 0.5;

        bool enable_joy_output = true;

        // {rotate, up/down, left/right shift, forward/backward shift, 0, 0}
        float m_axes[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        // {X, A(Arming), B(Guided), Y(Land), LB, RB, LT, RT, back, start, Button stick left, Button stick right} 
        int m_buttons[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};  

    public:
        Controller();
        void getParam();
        void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
        void toggleOutput();
        bool srvArming();
        bool srvGuided();
        bool srvLand();
        bool srvHover();
        bool srvGripper(bool open);
        void control();
};

Controller :: Controller(){
    sub_joy = n.subscribe<sensor_msgs::Joy>("joy", 1,  &Controller::joyCallback, this);
    pub_twist = n.advertise<geometry_msgs::Twist>("drone_twist", 10);
    
    srv_arming = n.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    srv_Guided = n.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    srv_land = n.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
    srv_hover = n.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    srv_param_set_ = n.serviceClient<mavros_msgs::ParamSet>("/mavros/param/set");
    srv_param_get_ = n.serviceClient<mavros_msgs::ParamGet>("/mavros/param/get");
    
}

void Controller :: getParam(){
    n.getParam("/drone_ardupilot_teleop/highspeed_linear",  m_highspeed_linear);
    n.getParam("/drone_ardupilot_teleop/highspeed_throttle", m_highspeed_throttle);
    n.getParam("/drone_ardupilot_teleop/highspeed_angular", m_highspeed_angular);

    n.getParam("/drone_ardupilot_teleop/lowspeed_linear",  m_lowspeed_linear);
    n.getParam("/drone_ardupilot_teleop/lowspeed_throttle", m_lowspeed_throttle);
    n.getParam("/drone_ardupilot_teleop/lowspeed_angular", m_lowspeed_angular);
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
        while(!state){ state = srvArming(); }
    }
    state = 0;
    if(m_buttons[2] == 1){ // joy B button
        while(!state){ state = srvGuided(); }
    }
    state = 0;
    if(m_buttons[3] == 1){ // joy Y button
        while(!state){ state = srvLand(); }
    }

    if(m_buttons[9] == 1){ // joy start button
        enable_joy_output = true;
        ROS_INFO("Joystick enable: %s", enable_joy_output ? "true" : "false");
    }

    if(m_buttons[8] == 1){ // joy back button
        enable_joy_output = true;
        ROS_INFO("Joystick enable: %s", enable_joy_output ? "true" : "false");
    }

    if(m_axes[5] == 1){ // joy arrow up
        srvGripper(true);
    }
    if(m_axes[5] == -1){ // joy arrow down
        srvGripper(false);
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

bool Controller :: srvGuided(){
    ROS_INFO("Srv Guided");
    mavros_msgs::SetMode srv_msg;
    srv_msg.request.custom_mode = "GUIDED";
    if(srv_Guided.call(srv_msg)){ return true; }
    else{ return false; }
}

bool Controller :: srvHover(){
    ROS_INFO("Srv Hover");
    mavros_msgs::SetMode srv_msg;
    srv_msg.request.custom_mode = "LOITER";
    if(srv_hover.call(srv_msg)){ return true; }
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

bool Controller :: srvGripper(bool open){
    std_srvs::Empty srv_msg;
    if(open){
        ROS_INFO("Srv Gripper on");
        // Call your gripper services here
    }else{
        ROS_INFO("Srv Gripper off");
        // Call your gripper services here
    }
    return true;
}



void Controller :: control(){
    ros::Rate rate(30);

    if(enable_joy_output){
        // twist
        geometry_msgs::Twist pub_msg_twist;
        // {rotate, up/down, left/right shift, forward/backward shift, 0, 0}

        float throttle, linear, angular;

        if(m_buttons[5] == 1){ // joy RB button
            throttle = m_lowspeed_throttle;
            linear  = m_lowspeed_linear;
            angular = m_lowspeed_angular; 
        }else{
            throttle = m_highspeed_throttle;
            linear  = m_highspeed_linear;
            angular = m_highspeed_angular;
        }
        if(m_buttons[4] == 1){ //joy LB button
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
