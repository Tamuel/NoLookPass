#include <ros/ros.h>
#include <no_look_pass_robot/estimation.h>
#include <no_look_pass_robot/master.h>
#include <sstream>
#include <cmath>
#include <deque>

#define DATA_SIZE 2

using namespace std;

no_look_pass_robot::estimation estimation_msg;

struct masterPosition
{
	double x;
	double y;
};
typedef struct masterPosition masterPos;


class routeEstimation
{
private:
	deque <masterPos> masterPos_deque;
	double master_path;

public:
	routeEstimation()
	{
		
	}
	
	~routeEstimation()
	{
		
	}

	void estimationfunc(no_look_pass_robot::master master_position);
};

void routeEstimation::estimationfunc(no_look_pass_robot::master master_position)
{
	masterPos push_temp;
	push_temp.x = master_position.x;
	push_temp.y = master_position.y;
	masterPos_deque.push_back(push_temp);
	
	if(masterPos_deque.size() > DATA_SIZE)
	{		
		double x1, x2, y1, y2, next_x, next_y;
		x1 = masterPos_deque.at(masterPos_deque.size() - 2).x;
		x2 = masterPos_deque.at(masterPos_deque.size() - 1).x;
		y1 = masterPos_deque.at(masterPos_deque.size() - 2).y;
		y2 = masterPos_deque.at(masterPos_deque.size() - 1).y;
		
		master_path = (y2 - y1) / (x2 - x1);
		
		next_x = 2*x2 - x1;
		next_y = master_path * next_x + y1 - master_path * x1;
		estimation_msg.x = next_x;
		estimation_msg.y = next_y;		
		estimation_msg.x_vel = (x2 - x1) / 0.1;
		estimation_msg.y_vel = (y2 - y1) / 0.1;

		masterPos_deque.pop_front();
	}
}



class masterRouteEstimationSubscriber
{
private:
	ros::NodeHandle _nh;
	ros::Subscriber _submasterRoute;
	routeEstimation* estimation;

public:
	masterRouteEstimationSubscriber()
	{
		_submasterRoute = _nh.subscribe(
			"/master", 100, &masterRouteEstimationSubscriber::masterRouteEstimation, this
		);
		estimation = new routeEstimation();
	}

	void masterRouteEstimation(const no_look_pass_robot::master& masterPositionData);

};


void masterRouteEstimationSubscriber::masterRouteEstimation(const no_look_pass_robot::master& masterPositionData)
{
	estimation->estimationfunc(masterPositionData);	
}


class masterRouteEstimationPublisher
{
private:
	ros::NodeHandle _nh;
	ros::Publisher _pub;

public:
	masterRouteEstimationPublisher()
	{
		_pub = _nh.advertise<no_look_pass_robot::estimation>("/estimation", 100, this);
	}

	void publish();
};

void masterRouteEstimationPublisher::publish()
{
	ros::Rate rate(10);

	while(ros::ok())
	{
		ros::spinOnce();
		_pub.publish(estimation_msg);
		estimation_msg.x = 0;
		estimation_msg.y = 0;		
		estimation_msg.x_vel = 0;
		estimation_msg.y_vel = 0;
		rate.sleep();
	}
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "masterRouteEstimation_node");

	masterRouteEstimationSubscriber* subscriber = new masterRouteEstimationSubscriber();
	masterRouteEstimationPublisher* publisher = new masterRouteEstimationPublisher();

	publisher->publish();

	return 0;
}
