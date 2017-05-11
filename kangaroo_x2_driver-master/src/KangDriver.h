#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>

#include "kangaroo_driver/kang_lib.hpp"

void joystickCB( const sensor_msgs::Joy::ConstPtr& js)
{
    ROS_INFO("CallBack: %f". js->axes[0]);
}

int main()
{
    ros::NodeHandle nh;
    ros::Subscriber js_sub;
    ros::Publisher  state_pub;

    int fd;
    


}