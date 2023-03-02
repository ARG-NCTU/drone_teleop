#include <iostream>
#include <math.h> 

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
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

        float m_max_linear = 3.0;
        float m_max_trottle = 2.0;
        float m_max_angular = 0.5;

        // {rotate, up/down, left/right shift, forward/backward shift, 0, 0}
        float m_axes[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        // {X, A(Arming), B(Offboard), Y(Land), LB, RB, LT, RT, back, start, Button stick left, Button stick right} 
        int m_buttons[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};  

    public:
        Controller();
        void getParam();
        void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
        bool srvArming();
        bool srvOffboard();
        bool srvLand();
        void control();
};

Controller :: Controller(){
    sub_joy = n.subscribe<sensor_msgs::Joy>("joy", 1,  &Controller::joyCallback, this);
    pub_twist = n.advertise<geometry_msgs::Twist>("drone_twist", 10);
    

    srv_arming = n.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    srv_offboard = n.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    srv_land = n.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
}

void Controller :: getParam(){
    n.getParam("/drone_teleop/max_linear", m_max_linear);
    n.getParam("/drone_teleop/max_trottle", m_max_trottle);
    n.getParam("/drone_teleop/max_angular", m_max_angular);
    return;
}


void Controller :: joyCallback(const sensor_msgs::Joy::ConstPtr& msg){
    for(int i=0; i<6; i++){ m_axes[i] = msg->axes[i]; }
    for(int i=0; i<12; i++){ m_buttons[i] = msg->buttons[i]; }
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



void Controller :: control(){
    ros::Rate rate(30);

    // mode & arming
    bool state = 0;
    if(m_buttons[1] == 1){
        while(!state){ state = srvArming(); }
    }
    state = 0;
    if(m_buttons[2] == 1){
        while(!state){ state = srvOffboard(); }
    }
    state = 0;
    if(m_buttons[3] == 1){
        while(!state){ state = srvLand(); }
    }

    // twist
    geometry_msgs::Twist pub_msg_twist;
    // {rotate, up/down, left/right shift, forward/backward shift, 0, 0}
    pub_msg_twist.angular.z = m_max_angular * m_axes[0];
    pub_msg_twist.linear.z = m_max_trottle * m_axes[1];
    pub_msg_twist.linear.y = m_max_linear * m_axes[2];
    pub_msg_twist.linear.x = m_max_linear * m_axes[3];

    pub_twist.publish(pub_msg_twist);

    rate.sleep();
    return;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "drone_teleop");
    Controller joy;
    while(ros::ok()){
        joy.control();
        ros::spinOnce();
    }
    return 0;
}