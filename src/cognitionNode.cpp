#include <ros/ros.h>
#include <no_look_pass_robot/move.h>
#include <no_look_pass_robot/laserScan.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <iomanip>

using namespace cv;

const int windowSize = 20;
const int robotSize = 40;
const int scanLength = 6; // Number of windows

ros::Publisher pub;

struct movePosition
{
	double x;
	double y;
};
typedef struct movePosition movePos;

movePos whereToGo()
{

}

void drawImage(int rangeSize,no_look_pass_robot::laserScan coord, float rangeMax)
{
	// draw the 'coord' in image plane
	const int nImageSize = 801;
	const int nImageHalfSize = nImageSize/2;
	const int nAxisSize = nImageSize/16;

	const int windowWidth = nImageSize/windowSize;
	const int windowHeight = nImageHalfSize/windowSize;
	bool windows[windowWidth][windowHeight] = {false};


	int imageCenterCooordX = nImageHalfSize;
	int imageCenterCooordY = nImageHalfSize;
	Mat image = Mat::zeros(nImageHalfSize, nImageSize, CV_8UC3);

	// Calculate windows
	for(int i=0; i < rangeSize; i++)
	{
		int x_coord = imageCenterCooordX + cvRound((coord.laser_data[i].x / rangeMax) * nImageSize);
		int y_coord = 0 + cvRound((coord.laser_data[i].y / rangeMax) * nImageSize);

		if(x_coord >= 0 && x_coord < nImageSize && y_coord >= 0 && y_coord < nImageHalfSize)
		{
			int windowXindex = x_coord / windowSize;
			int windowYindex = y_coord / windowSize;
			if(!windows[windowXindex][windowYindex])
				windows[windowXindex][windowYindex] = true;
		}
	}

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
	
	// Draw robot
	int robotWindowIndex = imageCenterCooordX / windowSize;
	int numberOfRobotBoxes = robotSize / windowSize;

	windows[robotWindowIndex][0] = false;
	rectangle(
		image,
		Point(robotWindowIndex * windowSize, 0),
		Point((robotWindowIndex + 1) * windowSize, windowSize),
		Scalar(0, 155, 255), CV_FILLED
	);

	int moveDestinationX = 0;
	int moveDestinationY = scanLength;
	
	// Move Message	
	no_look_pass_robot::move move_msg;

	bool detectObstacle = false;

	for(int i = 0; i < scanLength; i++)
	{
		// Detect Obstacles
		if(windows[robotWindowIndex][i])
		{
			int scanWidth = 1;
			// Scan Left and Right
			// First scan Left
			if(robotWindowIndex - scanWidth > 0)
			{
				// When can go
				if(!windows[robotWindowIndex - scanWidth][i])
				{
					moveDestinationX = -scanWidth;
					moveDestinationY = i;
					detectObstacle = true;
					printf("Left[%d, %d]\n", moveDestinationX, moveDestinationY);
				}
				// Need to scan more
			}
			// And Right
			if(robotWindowIndex + scanWidth < windowWidth)
			{
				// When can go
				if(!windows[robotWindowIndex + scanWidth][i])
				{
					moveDestinationX = scanWidth;
					moveDestinationY = i;
					detectObstacle = true;
					printf("Right[%d, %d]\n", moveDestinationX, moveDestinationY);
				}
				// Need to scan more
			}
			
			if(!detectObstacle) // Both side are blocked
			{
				moveDestinationX = 0;
				moveDestinationY = -1;
				detectObstacle = true;
				printf("Backward[%d, %d]\n", moveDestinationX, moveDestinationY);
			}
			if(detectObstacle)
				break;
		}
		else
		{
			rectangle(
				image,
				Point(robotWindowIndex * windowSize, i * windowSize),
				Point((robotWindowIndex + 1) * windowSize, (i + 1) * windowSize),
				Scalar(150, 150, 150), CV_FILLED
			);
		}
	}

	if(!(moveDestinationY == -1))
	{
		rectangle(
			image,
			Point((robotWindowIndex + moveDestinationX) * windowSize, moveDestinationY * windowSize),
			Point((robotWindowIndex + moveDestinationX + 1) * windowSize, (moveDestinationY + 1) * windowSize),
			Scalar(255, 50, 50), CV_FILLED
		);

		if(detectObstacle)
		{
			move_msg.relativeX = 0.5;
			move_msg.relativeY = -(float)moveDestinationX / (float)moveDestinationY * 2;
			printf("Avoid![%lf, %lf]\n", move_msg.relativeX, move_msg.relativeY);
		}
		else
		{
			printf("Forward[%d, %d]\n", moveDestinationX, moveDestinationY);
			move_msg.relativeX = 1.0;
			move_msg.relativeY = 0;
		}
	}
	else if(moveDestinationY == -1)
	{
		move_msg.relativeX = 0.1;
		move_msg.relativeY = 0.2;
	}

	pub.publish(move_msg);

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

void detectObstacles(const no_look_pass_robot::laserScan& laserScanData)
{
/*	for(int i = 0; i < laserScanData.laser_data.size(); i++)
	{
		printf("[%5.2lf,%5.2lf] ",
			laserScanData.laser_data[i].x, laserScanData.laser_data[i].y); 
		if(i != 0 && i % 5 == 0)
			printf("\n");
	}
*/
	drawImage(laserScanData.laser_data.size(), laserScanData, 10);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cognition_node");
	ros::NodeHandle nh;

	ros::Subscriber subLaserScan = nh.subscribe(
		"/laser_scan", 10, &detectObstacles
	);
	pub = nh.advertise<no_look_pass_robot::move>("/move_msg", 100);

	ros::Rate rate(10);

	// For testing ========================================================================
	movePos pos;

	while(ros::ok())
	{
		ros::spinOnce();

		//printf("Input [Relative X, RelativeY] : ");
		//scanf("%lf %lf", &pos.x, &pos.y);


		rate.sleep();
	}

	return 0;
}
