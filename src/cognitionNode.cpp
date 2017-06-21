#include <ros/ros.h>
#include <no_look_pass_robot/move.h>
#include <no_look_pass_robot/laserScan.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <iomanip>

using namespace cv;

struct movePosition
{
	double x;
	double y;
};
typedef struct movePosition movePos;

const int nImageSize = 801;
const int nImageHalfSize = nImageSize/2;
const int nAxisSize = nImageSize/16;

const int windowSize = 20;
const int robotSize = 40;
const int scanLength = 8; // Number of windows
const int scanHalfWidth = 3;

// Shared data by subscriber and publisher
no_look_pass_robot::move move_msg;



class obstacleDetector
{
private:
	int windowWidth;
	int windowHeight;
	bool **windows;

	int imageCenterCooordX;
	int imageCenterCooordY;
	Mat image;

	int robotWindowIndex;
	int numberOfRobotBoxes;
	bool windowWidthEven;


public:
	obstacleDetector()
	{
		windowWidth = nImageSize/windowSize;
		windowHeight = nImageHalfSize/windowSize;

		windows = new bool* [windowWidth];
		for(int i = 0; i < windowWidth; i++)
			windows[i] = new bool[windowHeight];

		imageCenterCooordX = nImageHalfSize;
		imageCenterCooordY = nImageHalfSize;

		robotWindowIndex = imageCenterCooordX / windowSize;
		numberOfRobotBoxes = robotSize / windowSize;

		if(windowWidth % 2 == 0)
			windowWidthEven = true;
		else
			windowWidthEven = false;

		obstacleDetector::clearImage();
	}
	
	~obstacleDetector()
	{
		for(int i = 0; i < windowWidth; i++)
			free(windows[i]);
		free(windows);
	}

	void clearImage();
	void clearWindows();
	void makeWindows(no_look_pass_robot::laserScan coord, float rangeMax);
	void drawWindows();
	void drawRobot();
	void showImage();
	void avoidObstacles();
};

void obstacleDetector::clearImage()
{
	for(int i = 0; i < windowWidth; i++)
		for(int j = 0; j < windowHeight; j++)
			windows[i][j] = false;
}

void obstacleDetector::clearWindows()
{
	image = Mat::zeros(nImageHalfSize, nImageSize, CV_8UC3);
}

void obstacleDetector::makeWindows(no_look_pass_robot::laserScan coord, float rangeMax)
{
	// Calculate windows
	for(int i = 0; i < coord.laser_data.size(); i++)
	{
		int x_coord =
			imageCenterCooordX + cvRound((coord.laser_data[i].x / rangeMax) * nImageSize);
		int y_coord =
			0 + cvRound((coord.laser_data[i].y / rangeMax) * nImageSize);

		if(x_coord >= 0 && x_coord < nImageSize && y_coord >= 0 && y_coord < nImageHalfSize)
		{
			int windowXindex = x_coord / windowSize;
			int windowYindex = y_coord / windowSize;
			if(!windows[windowXindex][windowYindex])
				windows[windowXindex][windowYindex] = true;
		}
	}
}

void obstacleDetector::drawWindows()
{
	// Draw windows
	for(int i = 0; i < windowHeight; i++)
	{
		for(int j = 0; j < windowWidth; j++)
		{
			if(!windows[j][i])
				rectangle(
					image,
					Point(j * windowSize, i * windowSize),
					Point((j + 1) * windowSize, (i + 1) * windowSize),
					Scalar(255, 255, 255), CV_FILLED
				);
		}
	}
}

void obstacleDetector::drawRobot()
{
	// Draw robot

	windows[robotWindowIndex][0] = false;
	rectangle(
		image,
		Point(robotWindowIndex * windowSize, 0),
		Point((robotWindowIndex + 1) * windowSize, windowSize),
		Scalar(0, 155, 255), CV_FILLED
	);
}

void obstacleDetector::showImage()
{
	line(
		image,
		Point(imageCenterCooordX, 0),
		Point(imageCenterCooordX+nAxisSize, 0),
		Scalar(0, 0, 255),
		2
	);
	line(
		image,
		Point(imageCenterCooordX, 0),
		Point(imageCenterCooordX, 0 + nAxisSize),
		Scalar(0, 255, 0),
		2
	);


	// image coordinate transformation
	flip(image, image, 0);

	imshow("Kinect LRF Preview", image);
	waitKey(30);
}

void obstacleDetector::avoidObstacles()
{
	float moveXweight = 0;
	float moveYweight = 0;
	float xWeightScale = 60; // Larger than turn less
	float yWeightScale = 2; // Larger than turn less
	int forwardScan = 1;

	for(int j = -scanHalfWidth; j < scanHalfWidth + (windowWidthEven ? 0 : 1); j++)
	{
		int i;
		for(i = forwardScan; i < scanLength + forwardScan; i++)
		{
			// Detect Obstacles
			if(windows[robotWindowIndex + j][i])
			{
				moveXweight +=
					-(float)((scanHalfWidth / (j + (windowWidthEven ? (j >= 0 ? 1 : 0) : 0))) *
					 (scanLength - i) / yWeightScale) / xWeightScale;
				break; 
			}
			else
			{
				rectangle(
					image,
					Point((robotWindowIndex + j) * windowSize, i * windowSize),
					Point((robotWindowIndex + j + 1) * windowSize, (i + 1) * windowSize),
					Scalar(150, 150, 150), CV_FILLED
				);
			}
		}
		moveYweight += i;
	}

	moveYweight /= (2 * scanHalfWidth * scanLength);
	moveYweight -= 0.5;
	moveYweight *= 2;
	if(moveYweight < 0.08 && moveYweight > -0.08)
		moveXweight = -2.0;
	

/*	rectangle(
		image,
		Point((robotWindowIndex + moveDestinationX) * windowSize,
			moveDestinationY * windowSize),
		Point((robotWindowIndex + moveDestinationX + 1) * windowSize,
			(moveDestinationY + 1) * windowSize),
		Scalar(255, 50, 50), CV_FILLED
	);*/

	move_msg.relativeX = moveYweight;
	move_msg.relativeY = -moveXweight;

	printf("Move[%lf, %lf]\n", move_msg.relativeX, move_msg.relativeY);
}

class cognitionSubscriber
{
private:
	ros::NodeHandle _nh;
	ros::Subscriber _subLaserScan;
	obstacleDetector* detector;

public:
	cognitionSubscriber()
	{
		_subLaserScan = _nh.subscribe(
			"/laser_scan", 10, &cognitionSubscriber::getLaserScanData, this
		);
		detector = new obstacleDetector();
	}

	void getLaserScanData(const no_look_pass_robot::laserScan& laserScanData);

};

void cognitionSubscriber::getLaserScanData(const no_look_pass_robot::laserScan& laserScanData)
{

/*	for(int i = 0; i < laserScanData.laser_data.size(); i++)
	{
		printf("[%5.2lf,%5.2lf] ",
			laserScanData.laser_data[i].x, laserScanData.laser_data[i].y); 
		if(i != 0 && i % 5 == 0)
			printf("\n");
	}
*/

	detector->clearImage();
	detector->clearWindows();
	detector->makeWindows(
		laserScanData,
		laserScanData.rangeMax
	);
	detector->drawWindows();
	detector->drawRobot();
	detector->avoidObstacles();
	detector->showImage();
}


class cognitionPublisher
{
private:
	ros::NodeHandle _nh;
	ros::Publisher _pub;

public:
	cognitionPublisher()
	{
		_pub = _nh.advertise<no_look_pass_robot::move>("/move_msg", 100, this);
	}

	void publish();
};

void cognitionPublisher::publish()
{
	ros::Rate rate(10);

	while(ros::ok())
	{
		ros::spinOnce();
		_pub.publish(move_msg);
		rate.sleep();
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cognition_node");

	move_msg.relativeX = 0;
	move_msg.relativeY = 0;

	cognitionSubscriber* subscriber = new cognitionSubscriber();
	cognitionPublisher* publisher = new cognitionPublisher();

	publisher->publish();

	return 0;
}
