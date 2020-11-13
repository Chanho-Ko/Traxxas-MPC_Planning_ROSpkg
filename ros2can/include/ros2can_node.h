#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "can_msgs/Frame.h"
#include <std_msgs/Float32MultiArray.h>


class Ros2Can
{
private:
    ros::NodeHandle nh_;
    ros::Publisher pub_can;
    ros::Subscriber sub_joy;
    ros::Subscriber sub_inputs;
    
    double axes[6];
    int buttons[12];
    double steer_gain = 0.045;
    double torque_gain = 0.2; //torque gain

public:
    Ros2Can(ros::NodeHandle* nodehandle);

	void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
    void planningCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void publish_CAN_Frame(const double steer_n, const double torque_n);
};

Ros2Can::Ros2Can(ros::NodeHandle* nodehandle) : nh_(*nodehandle)
{
    pub_can = nh_.advertise<can_msgs::Frame>("sent_messages", 1);
    sub_joy = nh_.subscribe("joy", 1, &Ros2Can::joyCallback, this);
    sub_inputs = nh_.subscribe("control_inputs", 1, &Ros2Can::planningCallback, this);
    
}

void Ros2Can::joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
    int i;
    for (i = 0; i < 6; i++)
        axes[i] = msg->axes[i];
    for (i = 0; i < 12; i++)
        buttons[i] = msg->buttons[i];

   
    publish_CAN_Frame(axes[3], axes[0]);
    
}

void Ros2Can::planningCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    double steer = msg->data[0];
    double torque = msg->data[1];

    steer *= 180/3.141592*steer_gain;
    if (steer > 1){
        steer = 1;
    }else if (steer < -1){
        steer = -1;
    }

    torque *= torque_gain;
    if (torque > 1){
        torque = 1;
    }else if (torque < -1){
        torque = -1;
    }

    publish_CAN_Frame(steer, torque);
    
}

void Ros2Can::publish_CAN_Frame(const double steer_n, const double torque_n)
{
    double pulsewidth1 = 1500. + 300.*steer_n; // motor input
    double pulsewidth2 = 1500. - 300.*torque_n; // steer input
    can_msgs::Frame canframe;
    canframe.id = 2;
    canframe.is_rtr = false;
    canframe.is_extended = false;
    canframe.is_error = false;
    canframe.dlc = 8;
    canframe.data[0] = (int)pulsewidth1 / 256;
    canframe.data[1] = (int)pulsewidth1 % 256;
    canframe.data[2] = (int)pulsewidth2 / 256;
    canframe.data[3] = (int)pulsewidth2 % 256;
    pub.publish(canframe);
}