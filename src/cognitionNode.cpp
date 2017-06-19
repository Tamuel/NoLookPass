#include <ros/ros.h>
#include <no_look_pass_robot/move.h>

struct movePosition
{
	double x;
	double y;
};
typedef struct movePosition movePos;

movePos whereToGo()
{

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cognition_node");
	ros::NodeHandle nh;

	ros::Publisher pub = nh.advertise<no_look_pass_robot::move>("/move_msg", 100);

	ros::Rate rate(10);
	int count = 0;

	// For testing ========================================================================
	movePos pos;

	while(ros::ok())
	{
		no_look_pass_robot::move move_msg;

		printf("Input [Relative X, RelativeY] : ");
		scanf("%lf %lf", &pos.x, &pos.y);

		move_msg.relativeX = pos.x;
		move_msg.relativeY = pos.y;
	
		pub.publish(move_msg);

		rate.sleep();
	}

	return 0;
}
