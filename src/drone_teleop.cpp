#include <iostream>
#include <math.h> 

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandTOL.h>
#include <behavior_tree/behavior_tree.h>

using namespace std; 


class Controller{
    private:
        ros::NodeHandle n;
        ros::Subscriber sub_joy;
        ros::Publisher pub_twist;

        ros::ServiceClient srv_arming;
        ros::ServiceClient srv_offboard;
        ros::ServiceClient srv_land;


        ros::ServiceClient srv_gripper0_on;
        ros::ServiceClient srv_gripper1_on;
        ros::ServiceClient srv_gripper2_on;
        ros::ServiceClient srv_gripper3_on;
        ros::ServiceClient srv_gripper4_on;
        ros::ServiceClient srv_gripper5_on;
        ros::ServiceClient srv_gripper6_on;
        ros::ServiceClient srv_gripper7_on;
        ros::ServiceClient srv_gripper8_on;


        ros::ServiceClient srv_gripper0_off;
        ros::ServiceClient srv_gripper1_off;
        ros::ServiceClient srv_gripper2_off;
        ros::ServiceClient srv_gripper3_off;
        ros::ServiceClient srv_gripper4_off;
        ros::ServiceClient srv_gripper5_off;
        ros::ServiceClient srv_gripper6_off;
        ros::ServiceClient srv_gripper7_off;
        ros::ServiceClient srv_gripper8_off;


        float m_highspeed_linear = 3.0;
        float m_highspeed_trottle = 2.0;
        float m_highspeed_angular = 1;

        float m_lowspeed_linear = 1.0;
        float m_lowspeed_trottle = 1.0;
        float m_lowspeed_angular = 0.5;

        bool enable_joy_output = true;

        // {rotate, up/down, left/right shift, forward/backward shift, 0, 0}
        float m_axes[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        // {X, A(Arming), B(Offboard), Y(Land), LB, RB, LT, RT, back, start, Button stick left, Button stick right} 
        int m_buttons[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};  

    public:
        bt::Condition condition;
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

Controller :: Controller() : condition("manual_trigger"){
    sub_joy = n.subscribe<sensor_msgs::Joy>("joy", 1,  &Controller::joyCallback, this);
    pub_twist = n.advertise<geometry_msgs::Twist>("drone_twist", 10);
    

    srv_arming = n.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    srv_offboard = n.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    srv_land = n.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");

    srv_gripper0_off = n.serviceClient<std_srvs::Empty>("drone_gripper/g0/off");
    srv_gripper1_off = n.serviceClient<std_srvs::Empty>("drone_gripper/g1/off");
    srv_gripper2_off = n.serviceClient<std_srvs::Empty>("drone_gripper/g2/off");
    srv_gripper3_off = n.serviceClient<std_srvs::Empty>("drone_gripper/g3/off");
    srv_gripper4_off = n.serviceClient<std_srvs::Empty>("drone_gripper/g4/off");
    srv_gripper5_off = n.serviceClient<std_srvs::Empty>("drone_gripper/g5/off");
    srv_gripper6_off = n.serviceClient<std_srvs::Empty>("drone_gripper/g6/off");
    srv_gripper7_off = n.serviceClient<std_srvs::Empty>("drone_gripper/g7/off");
    srv_gripper8_off = n.serviceClient<std_srvs::Empty>("drone_gripper/g8/off");

    srv_gripper0_on = n.serviceClient<std_srvs::Empty>("drone_gripper/g0/on");
    srv_gripper1_on = n.serviceClient<std_srvs::Empty>("drone_gripper/g1/on");
    srv_gripper2_on = n.serviceClient<std_srvs::Empty>("drone_gripper/g2/on");
    srv_gripper3_on = n.serviceClient<std_srvs::Empty>("drone_gripper/g3/on");
    srv_gripper4_on = n.serviceClient<std_srvs::Empty>("drone_gripper/g4/on");
    srv_gripper5_on = n.serviceClient<std_srvs::Empty>("drone_gripper/g5/on");
    srv_gripper6_on = n.serviceClient<std_srvs::Empty>("drone_gripper/g6/on");
    srv_gripper7_on = n.serviceClient<std_srvs::Empty>("drone_gripper/g7/on");
    srv_gripper8_on = n.serviceClient<std_srvs::Empty>("drone_gripper/g8/on");
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
    for(int i=0; i<6; i++){ m_axes[i] = msg->axes[i]; }
    for(int i=0; i<12; i++){ m_buttons[i] = msg->buttons[i]; }

    // mode & arming
    bool state = 0;
    if(m_buttons[0] == 1){
        condition.set(true);
        condition.publish();
    }
    state = 0;
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

    if(m_buttons[9] == 1){
        enable_joy_output = false;
        ROS_INFO("Joystick enable: %s", enable_joy_output ? "true" : "false");
    }

    if(m_buttons[8] == 1){
        enable_joy_output = true;
        ROS_INFO("Joystick enable: %s", enable_joy_output ? "true" : "false");
    }

    if(m_axes[5] == 1){
        srvGripper(true);
    }
    if(m_axes[5] == -1){
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

void Controller :: toggleOutput(){
    enable_joy_output = !enable_joy_output;
    ROS_INFO("Joystick enable: %s", enable_joy_output ? "true" : "false");
    return;
}

bool Controller :: srvGripper(bool open){
    std_srvs::Empty srv_msg;
    if(open){
        ROS_INFO("Srv Gripper on");
        srv_gripper0_on.call(srv_msg);
        srv_gripper1_on.call(srv_msg);
        srv_gripper2_on.call(srv_msg);
        srv_gripper3_on.call(srv_msg);
        srv_gripper4_on.call(srv_msg);
        srv_gripper5_on.call(srv_msg);
        srv_gripper6_on.call(srv_msg);
        srv_gripper7_on.call(srv_msg);
        srv_gripper8_on.call(srv_msg);
        // if(srv_gripper_on.call(srv_msg)){ return true; }
        // else{ return false; }
    }else{
        ROS_INFO("Srv Gripper off");
        srv_gripper0_off.call(srv_msg);
        srv_gripper1_off.call(srv_msg);
        srv_gripper2_off.call(srv_msg);
        srv_gripper3_off.call(srv_msg);
        srv_gripper4_off.call(srv_msg);
        srv_gripper5_off.call(srv_msg);
        srv_gripper6_off.call(srv_msg);
        srv_gripper7_off.call(srv_msg);
        srv_gripper8_off.call(srv_msg);
        // if(srv_gripper_off.call(srv_msg)){ return true; }
        // else{ return false; }
    }
    return true;
}



void Controller :: control(){
    ros::Rate rate(30);

    if(enable_joy_output){
        // twist
        geometry_msgs::Twist pub_msg_twist;
        // {rotate, up/down, left/right shift, forward/backward shift, 0, 0}

        float trottle, linear, angular;

        if(m_buttons[5] == 1){
            trottle = m_lowspeed_trottle;
            linear  = m_lowspeed_linear;
            angular = m_lowspeed_angular; 
        }else{
            trottle = m_highspeed_trottle;
            linear  = m_highspeed_linear;
            angular = m_highspeed_angular;
        }
        if(m_buttons[4] == 1){
            pub_msg_twist.angular.z = angular * m_axes[0];
            pub_msg_twist.linear.z = trottle * m_axes[1];

            pub_msg_twist.linear.y = linear * m_axes[2];
            pub_msg_twist.linear.x = linear * m_axes[3];
            

            pub_twist.publish(pub_msg_twist);
        }
        
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