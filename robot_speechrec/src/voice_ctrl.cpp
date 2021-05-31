#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <string>

std_msgs::String readCommand;

class RobotCtrl
{
public:
	RobotCtrl();

private:
	void commandCallBack(const std_msgs::String& command);
	void publish();
	int linearVelocity;
	int angularVelocity;
	ros::NodeHandle nodeHandle;
	ros::Publisher velocityPublisher;
	ros::Subscriber voiceRecognitionSubscriber;
	geometry_msgs::Twist cmd_vel;
};


RobotCtrl::RobotCtrl(): 
	nodeHandle("~"), linearVelocity(1), angularVelocity(0)
{
	nodeHandle.param("linear_velocity", linearVelocity, linearVelocity);
	nodeHandle.param("angular_velocity", angularVelocity, angularVelocity);
	velocityPublisher = nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	voiceRecognitionSubscriber = nodeHandle.subscribe("/recognizer/output", 1000, &RobotCtrl::commandCallBack, this);
	ros::Rate rate(5);

  	while (ros::ok()){
		//Publish cmd_vel 
		velocityPublisher.publish(cmd_vel);
		rate.sleep();
		ros::spinOnce();
	}
}


void RobotCtrl::commandCallBack(const std_msgs::String& command)
{ 
	if(command.data.compare("stop") == 0)
	{
		cmd_vel.angular.z = 0;
		cmd_vel.linear.x = 0;
	}
	else if(command.data.compare("forward") == 0)
	{
		cmd_vel.linear.x = 0.3;
		cmd_vel.angular.z = 0;
	}
	else if(command.data.compare("backward") == 0)
	{
		cmd_vel.linear.x = -0.3;
		cmd_vel.angular.z = 0;
	}
	else if(command.data.compare("left") == 0)
	{
		cmd_vel.linear.x = 0.0;
		cmd_vel.angular.z = 0.5;
	}
	else if(command.data.compare("right") == 0)
	{
		cmd_vel.linear.x = 0.0;
		cmd_vel.angular.z = -0.5;
	}
	else {
		cmd_vel.angular.z = 0;
		cmd_vel.linear.x = 0;
	}

	printf("cmd_vel.linear.x = %.2f, cmd_vel.angular.z = %.2f\n", cmd_vel.linear.x, cmd_vel.angular.z);

}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "voice_ctrl");
	RobotCtrl voice_teleop;
	ros::spin();
}
