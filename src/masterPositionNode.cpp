#include <ros/ros.h>
#include <no_look_pass_robot/depth.h>
#include <no_look_pass_robot/bounding.h>
#include <no_look_pass_robot/master.h>
#include <sstream>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

no_look_pass_robot::master master_msg;
using namespace message_filters;

typedef sync_policies::ExactTime<no_look_pass_robot::depth, no_look_pass_robot::bounding> MySyncPolicy;


class positionCalculation
{
private:


public:
	positionCalculation()
	{
		
	}
	
	~positionCalculation()
	{
		
	}
	void calculationfunc(no_look_pass_robot::depth::ConstPtr depthData, no_look_pass_robot::bounding::ConstPtr boundingData);
};

void positionCalculation::calculationfunc(no_look_pass_robot::depth::ConstPtr depthData, no_look_pass_robot::bounding::ConstPtr boundingData)
{
	int x = boundingData->x;
	int y = boundingData->y;
	int w = boundingData->w;
	int h = boundingData->h;

	int height = depthData->height;
	int width = depthData->width;

	float l = depthData->data[ 640 *(y + h / 2) + x + w/2 ];

	double master_x = ((float)x + (float)w/2 - (float)width / 2) / 100;
	

	if(l >= 0.7)
		l -= 0.7;

	double master_y = l;
	printf("%d, %d, %d, %d, %f, %f\n", x, y, w, h, master_x, master_y);
	master_msg.x = master_x;
	master_msg.y = master_y;
}



class masterPositionSubscriber
{
private:
	ros::NodeHandle _nh;
	message_filters::Subscriber<no_look_pass_robot::depth> *sub_depth;
	message_filters::Subscriber<no_look_pass_robot::bounding> *sub_bounding;
	Synchronizer<MySyncPolicy> *sync;
	positionCalculation* calculation;

public:
	masterPositionSubscriber()
	{		
		sub_depth = new message_filters::Subscriber<no_look_pass_robot::depth>(_nh, "/depth", 10);
		sub_bounding = new message_filters::Subscriber<no_look_pass_robot::bounding>(_nh, "/bounding", 10);

		sync = new Synchronizer<MySyncPolicy> (MySyncPolicy(10), *sub_depth, *sub_bounding);
  		sync->registerCallback(boost::bind(&masterPositionSubscriber::masterPosition, this, _1, _2));
		
		calculation = new positionCalculation();
	}

	void masterPosition(const no_look_pass_robot::depth::ConstPtr& depthData, const no_look_pass_robot::bounding::ConstPtr& boundingData);
};


void masterPositionSubscriber::masterPosition(const no_look_pass_robot::depth::ConstPtr& depthData, const no_look_pass_robot::bounding::ConstPtr& boundingData)
{
	if(depthData->data.size() > 0)
	{
		calculation->calculationfunc(depthData, boundingData);	
	}
}

class masterPositionPublisher
{
private:
	ros::NodeHandle _nh;
	ros::Publisher _pub;

public:
	masterPositionPublisher()
	{
		_pub = _nh.advertise<no_look_pass_robot::master>("/master", 1, this);
		master_msg.x = 0;
		master_msg.y = 0;
	}

	void publish();
};

void masterPositionPublisher::publish()
{
	ros::Rate rate(5);

	while(ros::ok())
	{
		ros::spinOnce();
		_pub.publish(master_msg);
		rate.sleep();
	}
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "masterPosition_node");

	masterPositionSubscriber* subscriber = new masterPositionSubscriber();
	masterPositionPublisher* publisher = new masterPositionPublisher();

	publisher->publish();

	return 0;
}
