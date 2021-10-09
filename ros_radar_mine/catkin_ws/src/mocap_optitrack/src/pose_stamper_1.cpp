#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"

//For each rigid body, a new pose_stamper node needs to be created to stamp its pose data.

geometry_msgs::Pose gp;
void rb1Callback(const geometry_msgs::Pose& p)
{	
	gp = p;
}
int main(int argc, char **argv)
{

	ros::init(argc, argv, "pose_stamper_1");
	ros::NodeHandle node;
	ros::Subscriber sub = node.subscribe("/rigid_body_1/pose",1, &rb1Callback);
	geometry_msgs::PoseStamped sp;
	ros::Publisher pub = node.advertise<geometry_msgs::PoseStamped>("/rigid_body_1/stamped", 10);	
	while(ros::ok()){
		sp.pose = gp;
		sp.header.stamp = ros::Time::now();
		sp.header.frame_id = "/rigid_body_1/base_link";
		pub.publish(sp);
		ros::spinOnce();
	}
	return 0;
}
