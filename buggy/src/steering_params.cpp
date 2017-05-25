#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <stdio.h>
#include <string>

ros::Publisher jt_pub;
ros::Subscriber sub;
geometry_msgs::Twist cmd_vel;

std::string type; 
int linearX,linearY,angularZ, brake, unlock, home;
bool initialized=false;



void JoyCallback( const sensor_msgs::Joy::ConstPtr& js)
{
	bool   unlockOn   = false;
	if(type=="gamepad") 
	{
		cmd_vel.linear.x  = js->axes[linearX]; //to/fro
		cmd_vel.linear.z  = js->buttons[brake];             //brake
		cmd_vel.linear.y  = js->buttons[unlock];            //unlock
		cmd_vel.angular.z = js->axes[angularZ]; //rotate left/right 
		cmd_vel.angular.x = js->buttons[home];    //homing
		
	}
	else if (type=="joystick") 
	{
		cmd_vel.linear.x  = js->axes[linearX]; //to/fro
		cmd_vel.linear.z  = js->buttons[brake];             //brake
		cmd_vel.linear.y  = js->buttons[unlock];            //unlock
		cmd_vel.angular.z = js->axes[angularZ]; //rotate left/right 
		cmd_vel.angular.x = js->axes[home];    //homing	
	}
	else 
	{
		cmd_vel.linear.x =  0.0; 
		cmd_vel.angular.z = 0.0;
		cmd_vel.linear.y  = 0.0; 
		ROS_INFO("No valid controller");	
	}
	
		unlockOn   = cmd_vel.linear.y;	
		if( cmd_vel.linear.z==1.0 )
		{
			cmd_vel.linear.x = 0.0;
		}

		if( unlockOn==0  )
		{
			cmd_vel.angular.z = 0.0;
			cmd_vel.linear.x  = 0.0;
		}	
	
	ROS_INFO("%s: cmd_vel: lx:%.3f,ly:%.3f,lz:%.3f,az:%.3f,ax:%.3f", type.c_str(), cmd_vel.linear.x, cmd_vel.linear.y,cmd_vel.linear.z, cmd_vel.angular.z, cmd_vel.angular.x);
	
	jt_pub.publish( cmd_vel );
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"buggy");
    ros::NodeHandle nh("steering");
    
    nh.param<std::string>("type", type,"null");
    nh.param("linearX",linearX,0);
    nh.param("linearY",linearY,3);
    nh.param("angularZ",angularZ,2);
    nh.param("brake", brake ,0);	
    nh.param("unlock",unlock,1);
    nh.param("home",home,0);
    
    cmd_vel.linear.x=0.0;
    cmd_vel.linear.y=0.0;
    cmd_vel.linear.z=0.0;
    cmd_vel.angular.x=0.0;
    cmd_vel.angular.y=0.0;
    cmd_vel.angular.z=0.0;
  
    jt_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);
    sub=nh.subscribe<sensor_msgs::Joy>("/joy",10, JoyCallback);

    ROS_INFO("%s: Axes: %d, %d, %d",type.c_str(),linearX, linearY, angularZ);
    
    while(ros::ok())
    {
    	ros::spinOnce();
    }
	
	
	return 0;
}
