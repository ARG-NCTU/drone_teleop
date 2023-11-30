#include <iostream>
#include <math.h> 

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandTOL.h>

using namespace std; 


class Controller{
    private:
        ros::NodeHandle n;
        ros::Subscriber sub_joy;
        ros::Publisher pub_twist;

        ros::ServiceClient srv_arming;
        ros::ServiceClient srv_offboard;
        ros::ServiceClient srv_land;

        float m_highspeed_linear = 3.0;
        float m_highspeed_trottle = 2.0;
        float m_highspeed_angular = 1;

        float m_lowspeed_linear = 1.0;
        float m_lowspeed_trottle = 1.0;
        float m_lowspeed_angular = 0.5;

        bool enable_joy_output = true;

        // {vr left_hand mushroom back/forward, left_hand mushroom left/right, left_hand trigger, left_hand grip}
        float m_axes[4] = {0.0, 0.0, 0.0, 0.0};
        // {VR X(Arming & Offboard), Y(Land), grip(for 2D move), trigger(for updown and heading)} 
        int m_buttons[4] = {0, 0, 0, 0};  

    public:
        Controller();
        void getParam();
        void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
        void toggleOutput();
        bool srvArming();
        bool srvOffboard();
        bool srvLand();
        bool srvGripper(bool open);
        void control();
};

Controller :: Controller(){
    sub_joy = n.subscribe<sensor_msgs::Joy>("vr_joy_drone", 1,  &Controller::joyCallback, this);
    pub_twist = n.advertise<geometry_msgs::Twist>("drone_twist", 10);
    

    srv_arming = n.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    srv_offboard = n.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    srv_land = n.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
}

void Controller :: getParam(){
    n.getParam("/drone_teleop/highspeed_linear",  m_highspeed_linear);
    n.getParam("/drone_teleop/highspeed_trottle", m_highspeed_trottle);
    n.getParam("/drone_teleop/highspeed_angular", m_highspeed_angular);

    n.getParam("/drone_teleop/lowspeed_linear",  m_lowspeed_linear);
    n.getParam("/drone_teleop/lowspeed_trottle", m_lowspeed_trottle);
    n.getParam("/drone_teleop/lowspeed_angular", m_lowspeed_angular);
    return;
}


void Controller :: joyCallback(const sensor_msgs::Joy::ConstPtr& msg){
    for(int i=0; i<4; i++){ m_axes[i] = msg->axes[i]; }
    for(int i=0; i<4; i++){ m_buttons[i] = msg->buttons[i]; }

    // mode & arming
    bool state_arm = 0;
    bool state_offboard = 0;
    bool state_land = 0;
    
    if(m_buttons[0] == 1){
        //VR left hand button X
        while(!state_arm && !state_offboard){ 
            state_offboard = srvOffboard();
            state_arm = srvArming(); 
            
        }
    }
    state_arm = 0;
    state_offboard = 0;
    if(m_buttons[1] == 1){
        //VR left hand button Y
        while(!state_land){ state_land = srvLand(); }
    }
    state_land = 0;

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
    pub_msg_twist.linear.x = 0.0;
    pub_msg_twist.linear.y = 0.0;
    pub_msg_twist.linear.z = 0.0;

    pub_msg_twist.angular.x = 0.0;
    pub_msg_twist.angular.y = 0.0;
    pub_msg_twist.angular.z = 0.0;
    pub_twist.publish(pub_msg_twist);
    
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

void Controller :: toggleOutput(){
    enable_joy_output = !enable_joy_output;
    ROS_INFO("Joystick enable: %s", enable_joy_output ? "true" : "false");
    return;
}



void Controller :: control(){
    ros::Rate rate(30);
    enable_joy_output = true;
    if(enable_joy_output){
        // twist
        geometry_msgs::Twist pub_msg_twist;
        // {rotate, up/down, left/right shift, forward/backward shift, 0, 0}

        float trottle, linear, angular;

        // if(m_buttons[5] == 1){
        //     trottle = m_lowspeed_trottle;
        //     linear  = m_lowspeed_linear;
        //     angular = m_lowspeed_angular; 
        // }else{
        //     trottle = m_highspeed_trottle;
        //     linear  = m_highspeed_linear;
        //     angular = m_highspeed_angular;
        // }
        trottle = m_highspeed_trottle;
        linear  = m_highspeed_linear;
        angular = m_highspeed_angular;
        if(m_buttons[2] == 1 && m_buttons[3] == 1){
            //if (updown heading) and (2D move) enable at the same time, move 2D and updown heading (no heading)
            pub_msg_twist.linear.z = trottle * m_axes[0];
            pub_msg_twist.linear.x = linear * m_axes[2];
            pub_msg_twist.linear.y = linear * m_axes[3];
            
        }
        else if(m_buttons[2] == 1){
            //vr left hand grip button (2D move)
            pub_msg_twist.linear.x = linear *m_axes[2];
            pub_msg_twist.linear.y = linear *m_axes[3] ;
            
        }
        else if(m_buttons[3] == 1){
            //vr left hand trigger button (updown heading)
            pub_msg_twist.linear.z = trottle * m_axes[0];
            pub_msg_twist.angular.z = angular * -m_axes[1];
        }

        if(m_buttons[2] == 1 || m_buttons[3] == 1){
            pub_twist.publish(pub_msg_twist);
        }
        
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