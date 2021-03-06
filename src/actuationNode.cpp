#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <boost/thread/mutex.hpp>
#include <tf/tf.h>
#include <no_look_pass_robot/move.h>


#define toRadian(degree)	((degree) * (M_PI / 180.))
#define toDegree(radian)	((radian) * (180. / M_PI))

// Global variable
boost::mutex mutex;
nav_msgs::Odometry g_odom;

const double maximumSpeed = 0.2;
const double minimumSpeed = 0.07;

bool interrupt = false;

// odom
tf::Transform
getCurrentTransformation(void)
{
	// transformation
	tf::Transform transformation;

	// odom
	nav_msgs::Odometry odom;

	// copy a global '/odom' message with the mutex
	mutex.lock(); {
		odom = g_odom;
	} mutex.unlock();

	//
	transformation.setOrigin(
		tf::Vector3(
			odom.pose.pose.position.x,
			odom.pose.pose.position.y,
			odom.pose.pose.position.z
		)
	);

	//
	transformation.setRotation(
		tf::Quaternion(
			odom.pose.pose.orientation.x,
			odom.pose.pose.orientation.y,
			odom.pose.pose.orientation.z,
			odom.pose.pose.orientation.w
		)
	);

	//
	return transformation;
}



tf::Transform
getInitialTransformation(void)
{
	tf::Transform transformation;

	ros::Rate loopRate(1000.0);

	while(ros::ok()) {
		ros::spinOnce();

		transformation = getCurrentTransformation();

		if(transformation.getOrigin().getX() != 0. ||
		transformation.getOrigin().getY() != 0. &&
		transformation.getOrigin().getZ() != 0.)
		{
			break;
		} else {
			loopRate.sleep();
		}
	}

	return transformation;
}


// 터틀봇의 좌표계를 기준으로 (relativeX, relativeY) 좌표로 이동. 터틀봇의 시점에서 생각 필요
bool moveToRelativeLocation(
	ros::Publisher &pubTeleop,
	double relativeX,
	double relativeY,
	double dTranslationSpeed,
	double dRotationSpeed
	)
{
	tf::Transform initialTransformation = getInitialTransformation();
	geometry_msgs::Twist baseCmd;
	baseCmd.linear.x = dTranslationSpeed;
	baseCmd.linear.y = 0.0;
	bool bDone = false;
	ros::Rate loopRate(1000.0);

	double backward = 1;

	// get current transformation
	tf::Transform currentTransformation;
	//see how far we've traveled
	tf::Transform relativeTransformation;
	tf::Quaternion rotationQuat;

	if(relativeX >= 0)
		backward = 1;
	else
		backward = -1;

	// Start moving
	printf("Start move to relative location : [%f, %f]\n", relativeX, relativeY);
	interrupt = false;

	while(ros::ok() && !bDone) {
		ros::spinOnce();

		currentTransformation = getCurrentTransformation();
		relativeTransformation = initialTransformation.inverse() * currentTransformation;
		rotationQuat = relativeTransformation.getRotation();


		double dHaveToMoveDist = sqrt(
			pow(fabs(relativeTransformation.getOrigin().getX() - relativeX), 2) +
			pow(fabs(relativeTransformation.getOrigin().getY() - relativeY), 2)
		);
		double dAngleTurned = atan2(
			(2 * rotationQuat[2] * rotationQuat[3]),
			(1-(2 * (rotationQuat[2] * rotationQuat[2])
			))
		);
		double dRotation = atan2(
			(relativeY - relativeTransformation.getOrigin().getY()) * backward,
			(relativeX - relativeTransformation.getOrigin().getX()) * backward
		);
//		printf("Rota : %f, Move : %f, Rot : %f, Angle : %f\n",
//			toDegree(dRotation - dAngleTurned), dHaveToMoveDist,
//			dRotation, dAngleTurned);

		double haveToRotate = dRotation - dAngleTurned;
		if(dHaveToMoveDist < 0.05 || interrupt) 
		{
			bDone = true;
			interrupt = true;
			printf("Move Done!\n");
			break;
		} else {
			if(haveToRotate < 0)
				baseCmd.angular.z = -dRotationSpeed * fabs(haveToRotate) * 2;
			else
				baseCmd.angular.z = dRotationSpeed * fabs(haveToRotate) * 2;

			baseCmd.linear.x = dTranslationSpeed * dHaveToMoveDist * 3 * backward / (fabs(haveToRotate) * 2);

			if(baseCmd.linear.x * backward < minimumSpeed)
				baseCmd.linear.x = minimumSpeed * backward;
			if(baseCmd.linear.x * backward > maximumSpeed)
				baseCmd.linear.x = maximumSpeed * backward;

			//send the drive command
			pubTeleop.publish(baseCmd);

			// sleep!
			loopRate.sleep();
		}


	}

	baseCmd.linear.x = 0.0;
	baseCmd.angular.z = 0.0;
	pubTeleop.publish(baseCmd);

	return bDone;
}

class actuationPublisher
{
private:
	ros::NodeHandle _nh;
	ros::Publisher _pub_teleop;

public:
	actuationPublisher()
	{
		_pub_teleop = _nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 100, this);

	}
	
	ros::Publisher getPublisher()
	{
		return _pub_teleop;
	}
};

class actuationSubscriber
{
private:
	ros::NodeHandle _nh;
	ros::NodeHandle _movenh;
	ros::Subscriber _sub_odom;
	ros::Subscriber _sub_move;
	actuationPublisher* _publisher;

	
	// Get Odometry Callback function
	void odomMsgCallback(const nav_msgs::Odometry &msg)
	{
		// receive a '/odom' message with the mutex
		mutex.lock(); {
			g_odom = msg;
		} mutex.unlock();
	}

	// Get Move Message callback function
	void moveMsgCallback(const no_look_pass_robot::move &msg)
	{
		double relativeX = msg.relativeX;
		double relativeY = msg.relativeY;
		ros::Publisher _pub = _publisher->getPublisher();
		moveToRelativeLocation(_pub, relativeX, relativeY, 0.1, 0.5);
	}

public:
	actuationSubscriber(actuationPublisher* publisher)
	{
		_publisher = publisher;
		
		// Decleation of subscriber
		_sub_odom = _nh.subscribe("/odom", 100, &actuationSubscriber::odomMsgCallback, this);
		_sub_move = _movenh.subscribe("/move_msg", 100, &actuationSubscriber::moveMsgCallback, this);
	}
	
	void subscribe()
	{
		ros::Rate rate (100); // 10
		while(ros::ok())
		{
			ros::spinOnce();
			rate.sleep();
		}
	}
};



int main(int argc, char **argv)
{
	ros::init(argc, argv, "actuation_node");

	// Create a publisher object
	actuationPublisher* publisher = new actuationPublisher();

	// Create a subscriber object
	actuationSubscriber* subscriber = new actuationSubscriber(publisher);
	subscriber->subscribe();


	return 0;
}
