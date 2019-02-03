#include <ros/ros.h>
#include <no_look_pass_robot/move.h>
#include <no_look_pass_robot/laserScan.h>
#include <opencv/cv.h>
#include <no_look_pass_robot/master.h>
#include <no_look_pass_robot/estimation.h>
#include <opencv/highgui.h>
#include <nav_msgs/Odometry.h>
#include <cv_bridge/cv_bridge.h>
#include <iomanip>
#include <ctime>
#include <boost/thread/mutex.hpp>
#include <iostream>

#define INF 9999
#define WANT_X true
#define WANT_Y false

using namespace cv;

enum windowState {FREE, BLOCK, VISIT};
enum coord {X, Y};

// Global variable
boost::mutex mutex;
nav_msgs::Odometry g_odom;

struct intCoord
{
	int x;
	int y;
};
typedef struct intCoord iCoord;

struct lineFunc
{
	float slope;
	float c;
};
typedef struct lineFunc lineF;

const int nImageHSize = 801;
const int nImageVSize = nImageHSize/2;
const int nAxisSize = nImageHSize/16;

// 계산 편의를 위해 window 갯수, robot의 window 갯수는 짝수가 되도록 설정해야함
const int windowSize = 20;
const int robotSize = 40;
const int scanLength = 10; // Number of windows
const int frontSideBlockLength = 2;

// Shared data by subscriber and publisher
no_look_pass_robot::move move_msg;


class obstacleDetector
{
private:
	int windowWidth;
	int windowHeight;
	short **windows;
	vector<iCoord> edgeWindows;

	int imageCenterCooordX;
	int imageCenterCooordY;
	Mat image;

	int robotWindowIndex;
	int nRobotWindows;
	bool windowWidthEven;


public:
	obstacleDetector()
	{
		windowWidth = nImageHSize/windowSize;
		windowHeight = nImageVSize/windowSize;

		windows = new short* [windowWidth];
		for(int i = 0; i < windowWidth; i++)
			windows[i] = new short[windowHeight];

		imageCenterCooordX = nImageVSize;
		imageCenterCooordY = nImageVSize;

		robotWindowIndex = imageCenterCooordX / windowSize;
		nRobotWindows = robotSize / windowSize;
		nRobotWindows += (robotSize / windowSize - (float)nRobotWindows) > 0.5 ? 1 : 0; // 반올림 

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
	
	float getLength(float x1, float y1, float x2, float y2);
	float getSlope(float x, float y);
	float getSlope(float x1, float y1, float x2, float y2);
	lineF getLineFunction(float x, float y);
	lineF getLineFunction(float x1, float y1, float x2, float y2);
	iCoord getPointFromLine(bool wantIsX, float xOrY, lineF line);
	void clearImage();
	void clearWindows();
	void makeWindows(no_look_pass_robot::laserScan coord, float rangeMax);
	void drawWindows();
	void showImage();
	void avoidObstacles(float destinationX, float destinationY);
	bool detectObstacles(int xIndex, int yIndex, iCoord* detectedPos);
	iCoord lineScaning(float x1, float y1, float x2, float y2);
};

float obstacleDetector::getLength(float x1, float y1, float x2, float y2)
{
	return sqrt(pow((double)(x1 - x2), 2) + pow((double)(y1 - y2), 2));
}

lineF obstacleDetector::getLineFunction(float x, float y)
{
	lineF newLineFunc;

	newLineFunc.slope = getSlope(x, y);
	newLineFunc.c = 0;

	return newLineFunc;
}

lineF obstacleDetector::getLineFunction(float x1, float y1, float x2, float y2)
{
	lineF newLineFunc;

	newLineFunc.slope = getSlope(x1, y1, x2, y2);
	newLineFunc.c = y1 - newLineFunc.slope * x1;

	return newLineFunc;
}

iCoord obstacleDetector::getPointFromLine(bool wantIsX, float xOrY, lineF line)
{
	iCoord point;
	if (wantIsX)
	{
		point.x = (xOrY - line.c) / line.slope;
		point.y = xOrY;
	}
	else
	{
		point.x = xOrY;
		point.y = line.slope * xOrY + line.c;
	}

	return point;
}

float obstacleDetector::getSlope(float x, float y)
{
	return (x == 0 ? INF :  y / x);
}

float obstacleDetector::getSlope(float x1, float y1, float x2, float y2)
{
	return (x1 - x2 == 0 ? INF : (y1 - y2) / (x1 - x2));
}

void obstacleDetector::clearImage()
{
	for(int i = 0; i < windowWidth; i++)
		for(int j = 0; j < windowHeight; j++)
			windows[i][j] = FREE;
}

void obstacleDetector::clearWindows()
{
	image = Mat::zeros(nImageVSize, nImageHSize, CV_8UC3);
}

void obstacleDetector::makeWindows(no_look_pass_robot::laserScan coord, float rangeMax)
{
	int startEdge[2] = { -1, -1 };
	int lastBlockedWindow[2] = { 0, 0 };
	edgeWindows.clear();
	// Calculate windows
	for(int i = 0; i < coord.laser_data.size(); i++)
	{
		int x_coord =
			imageCenterCooordX + cvRound((coord.laser_data[i].x / rangeMax) * nImageHSize);
		int y_coord =
			0 + cvRound((coord.laser_data[i].y / rangeMax) * nImageHSize);

		if(x_coord >= 0 && x_coord < nImageHSize && y_coord >= 0 && y_coord < nImageVSize)
		{
			int windowXindex = x_coord / windowSize;
			int windowYindex = y_coord / windowSize;
			if(windowXindex >= 0 && windowXindex < windowWidth && windowYindex >= 0 && windowYindex < windowHeight)
			{
				if(windows[windowXindex][windowYindex] == FREE &&
				!(windowYindex == 0 && windowXindex == robotWindowIndex))
				{
					windows[windowXindex][windowYindex] = BLOCK;
					if (startEdge[X] == -1 && startEdge[Y] == -1)
					{
						startEdge[X] = windowXindex;
						startEdge[Y] = windowYindex;
					}
					// 분리되어 있을 경우
					else if (abs(windowXindex - lastBlockedWindow[0]) > 1 || abs(windowYindex - lastBlockedWindow[1]) > 1)
					{
						iCoord pushWindow = {
							startEdge[X], startEdge[Y]
						};
						iCoord pushWindow2 = {
							lastBlockedWindow[X], lastBlockedWindow[Y]
						};
						edgeWindows.push_back(pushWindow);
						edgeWindows.push_back(pushWindow2);
						startEdge[X] = windowXindex;
						startEdge[Y] = windowYindex;
					}
					lastBlockedWindow[X] = windowXindex;
					lastBlockedWindow[Y] = windowYindex;
				}
			}
		}
	}

	if(!(startEdge[0] == -1 && startEdge[1] == -1))
	{
		iCoord pushWindow = {
			startEdge[X], startEdge[Y]
		};
		iCoord pushWindow2 = {
			lastBlockedWindow[X], lastBlockedWindow[Y]
		};
		edgeWindows.push_back(pushWindow);
		edgeWindows.push_back(pushWindow2);
	}

	for(int i = 0; i < frontSideBlockLength; i++)
	{
		windows[robotWindowIndex - nRobotWindows / 2 - 2][i] = BLOCK;
		windows[robotWindowIndex + nRobotWindows / 2 + 1][i] = BLOCK;
	}
}

void obstacleDetector::drawWindows()
{
	// Draw windows
	for(int i = 0; i < windowHeight; i++)
	{
		for(int j = 0; j < windowWidth; j++)
		{
			Scalar color;
			switch(windows[j][i])
			{
				case FREE:
					color = Scalar(255, 255, 255);
				break;

				case BLOCK:
					color = Scalar(0, 0, 0);
				break;

				case VISIT:
					color = Scalar(120, 120, 120);
				break;
				
			}
			rectangle(
				image,
				Point(j * windowSize, i * windowSize),
				Point((j + 1) * windowSize, (i + 1) * windowSize),
				color, CV_FILLED
			);
		}
	}
}

void obstacleDetector::showImage()
{

	// image coordinate transformation
	flip(image, image, 0);

	imshow("Kinect LRF Preview", image);
	waitKey(30);
}

bool obstacleDetector::detectObstacles(int xIndex, int yIndex, iCoord* detectedPos)
{
	if(xIndex >= 0 && xIndex < windowWidth && yIndex >= 0 && yIndex < windowHeight)
	{
		for(int j = -nRobotWindows / 2 + 1;  j <= nRobotWindows / 2; j++)
		{
			for(int k = -nRobotWindows / 2; k < nRobotWindows / 2; k++)
			{
				if(xIndex + k >= 0 && xIndex + k < windowWidth && yIndex + j >= 0 && yIndex + j < windowHeight)
				{
					if(windows[xIndex + k][yIndex + j] == BLOCK)
					{
						(*detectedPos).x = xIndex + k;
						(*detectedPos).y = yIndex + j;
						return true;
					}
					else
					{
						// Check visit
						windows[xIndex + k][yIndex + j] = VISIT;
						rectangle(
							image,
							Point((xIndex + k) * windowSize, (yIndex + j) * windowSize),
							Point((xIndex + k + 1) * windowSize, (yIndex + j + 1) * windowSize),
							Scalar(220, 220, 220), CV_FILLED
						);
					}
				}
			}
		}
	}
	return false;
}

iCoord obstacleDetector::lineScaning(float x1, float y1, float x2, float y2)
{
	lineF lineFunction = getLineFunction(x1, y1, x2, y2);
	iCoord detectedPos;
	detectedPos.x = -1; detectedPos.y = -1;
	if (fabs(lineFunction.slope) >= 1)
	{
		if(y1 < y2)
		{
			for(int y = y1; y < y2; y++)
				if(detectObstacles(getPointFromLine(WANT_X, y, lineFunction).x, y, &detectedPos))
					return detectedPos;
		}
		else
		{
			for(int y = y2; y < y1; y++)
				if(detectObstacles(getPointFromLine(WANT_X, y, lineFunction).x, y, &detectedPos))
					return detectedPos;
		}
	}
	else
	{
		if(lineFunction.slope >= 0)
		{
			if(x1 < x2)
			{
				for(int x = x1; x <= x2; x++)
					if(detectObstacles(x, getPointFromLine(WANT_Y, x, lineFunction).y, &detectedPos))
						return detectedPos;
			}
			else
			{
				for(int x = x2; x <= x1; x++)
					if(detectObstacles(x, getPointFromLine(WANT_Y, x, lineFunction).y, &detectedPos))
						return detectedPos;
			}
		}
		else
		{
			if(x1 > x2)
			{
				for(int x = x1; x >= x2; x--)
					if(detectObstacles(x, getPointFromLine(WANT_Y, x, lineFunction).y, &detectedPos))
						return detectedPos;
			}
			else
			{
				for(int x = x2; x >= x1; x--)
					if(detectObstacles(x, getPointFromLine(WANT_Y, x, lineFunction).y, &detectedPos))
						return detectedPos;
			}
		}
	}

	return detectedPos;
}

void obstacleDetector::avoidObstacles(float destinationX, float destinationY)
{
	// Set Destination Window and make path =========================================
	lineF lineFunc = getLineFunction(robotWindowIndex, 0, destinationX + robotWindowIndex, destinationY);
	iCoord destinationPos;
	destinationPos.x = 0; destinationPos.y = 0;
	float halfWindowSize = windowSize / 2;
	printf("Des Slope : %f, C : %f\n", lineFunc.slope, lineFunc.c);

	// Calculate Destination window
	// 각 윈도우의 중심 좌표를 이용해 계산
	if(fabs(lineFunc.slope) > (float)(nImageHSize / 2) / (float)nImageVSize)
	{
		// When destination window in top column
		destinationPos.y = windowHeight - 1;
		destinationPos.x = getPointFromLine(WANT_X, destinationPos.y, lineFunc).x;
	}
	else
	{
		// When destination window in sidemost row
		if (lineFunc.slope > 0)
			destinationPos.x = windowWidth - 1;
		else
			destinationPos.x = 0;

		destinationPos.y = getPointFromLine(WANT_Y, destinationPos.x, lineFunc).y;
	}
	destinationPos.x = destinationX + robotWindowIndex;
	destinationPos.y = destinationY;
	if(destinationX >= 0 && destinationX < windowWidth && destinationY >= 0 && destinationY < windowWidth)
	{
		windows[destinationPos.x][destinationPos.y] = FREE;
		windows[destinationPos.x - 1][destinationPos.y] = FREE;
		windows[destinationPos.x][destinationPos.y + 1] = FREE;
		windows[destinationPos.x - 1][destinationPos.y + 1] = FREE;
		printf("des X, Y : %d, %d\n", destinationPos.x, destinationPos.y);
	}
	else
	{
		destinationX = robotWindowIndex + 1;
		destinationY = 0;
	}

	// Make Path to destination (by use edgeWindows)
	vector<iCoord> path;
	iCoord fisrtVertex = { robotWindowIndex, 0 };
	path.push_back(fisrtVertex);
	
	bool detectObstacle = false;
	bool canGo = true;

	for (int i = 0; i < path.size(); i++)
	{
		iCoord currentPos;
		currentPos.x = path.back().x; currentPos.y = path.back().y;
		iCoord detectedPos;
		detectedPos = lineScaning(currentPos.x, currentPos.y, destinationPos.x, destinationPos.y);
		printf("path [%d], Detect : %d, %d\n", i, detectedPos.x, detectedPos.y);

		// 직선으로 선을 그어 충돌 체크
		if(!(detectedPos.x == -1 && detectedPos.y == -1)) // When collide
		{
			// 가까운 순서대로 edgeWindow 정렬
			vector<int> sortedEdgesIndex;
			const int checkSize = edgeWindows.size();
			bool check[checkSize];
			for(int k = 0; k < edgeWindows.size(); k++)
			{
				float smallestLength = INF;
				float currentLength = 0;
				int smallest = 0;
				for(int j = 0; j < edgeWindows.size(); j++)
				{
					if(!check[j])
					{
						currentLength = getLength(detectedPos.x, detectedPos.y, edgeWindows[j].x, edgeWindows[j].y);
						if(currentLength < smallestLength)
						{
							smallestLength = currentLength;
							smallest = j;
						}
					}
				}
				check[smallest] = true;
				sortedEdgesIndex.push_back(smallest);
				
			}
			
			// 가장 가까운 edgeWindow 부터 탐색
			iCoord checkCollision;
			int lookSideMore;
			int lookBottomMore;
			int times = 0;
			canGo = false;
			try
			{
				for(int k = 0; k < 10; k++)
				{
					if(k % 5 == 0)
						times += 1;

					switch(k % 5)
					{
						case 0:
							lookSideMore = 1;
							lookBottomMore = 0;
							break;
						case 1:
							lookSideMore = 1;
							lookBottomMore = 1;
							break;
						case 2:
							lookSideMore = 0;
							lookBottomMore = 1;
							break;
						case 3:
							lookSideMore = 1;
							lookBottomMore = -1;
							break;
						case 4:
							lookSideMore = 0;
							lookBottomMore = -1;
							break;
					}
					for(int j = 0; j < sortedEdgesIndex.size(); j++)
					{
						int leftOrRight = 0;
						if(sortedEdgesIndex[j] % 2 == 0) // when on right side
							leftOrRight = (nRobotWindows + lookSideMore);
						else
							leftOrRight = -(nRobotWindows - lookSideMore);
						iCoord edgePos;
						if(sortedEdgesIndex[j] < edgeWindows.size() && sortedEdgesIndex[j] > 0)
						{
							edgePos.x = edgeWindows[sortedEdgesIndex[j]].x + leftOrRight * times;
							edgePos.y = edgeWindows[sortedEdgesIndex[j]].y - lookBottomMore * times;
						}
						iCoord temp;
						if(!detectObstacles(edgePos.x, edgePos.y, &temp))
						{
							checkCollision = lineScaning(currentPos.x, currentPos.y, edgePos.x, edgePos.y);
							if(checkCollision.x == -1 && checkCollision.y == -1)
							{
								bool notSame = true;
								for(int s = 0; s < path.size(); s++)
									if(path[s].x == edgePos.x && path[s].y == edgePos.y)
									{
										notSame = false;
										break;
									}
								if(notSame)
								{
									canGo = true;
									path.push_back(edgePos);
									printf("Push Path : %d, %d [%d]\n", edgePos.x, edgePos.y, (int)path.size());
									break;
								}
								else
								{
									break;
								}
							}
						}
					}
					if(canGo)
						break;
				}
			}
			catch(const std::exception& e)
			{
				std::cout << e.what() << std::endl;
			}

			// 갈곳이 없을 경우
			if(!canGo)
				break;
		}
		else // 충돌 X
		{
			path.push_back(destinationPos);
			break;
		}
		if(i > 10)
			break;
	}

	// 필요 없는 길 삭제
	try
	{
		if(canGo)
		{
			iCoord currentPos;
			iCoord nextPos;
			iCoord detectedPos;
			for(int k = 0; k < 2; k++)
			{
				for(int i = 0; i < path.size(); i++)
				{
					currentPos = path[i];
					int j = 1;
					for(; i + j < path.size(); j++)
					{
						nextPos = path[i + j];
						detectedPos = lineScaning(currentPos.x, currentPos.y, nextPos.x, nextPos.y);

						if(!(detectedPos.x == -1 && detectedPos.y == -1)) // When collide
							break;
					}
					for(int k = i + 1; k < i + j - 1; k++)
						path.erase(path.begin() + i + 1);
				}
			}
		}
		else
		{
			while(path.size() > 2)
				path.erase(path.begin() + 1);
		}
	}
	catch(const std::exception& e)
	{
		std::cout << e.what() << std::endl;
	}

		
		
	for(int i = 1; i < path.size(); i++)
	{
		line(
			image,
			Point(path[i - 1].x * windowSize, (path[i - 1].y + 1) * windowSize),
			Point(path[i].x * windowSize, (path[i].y + 1) * windowSize),
			Scalar(50, 50, 150),
			2
		);
	}
	
	for(int i = 0; i < path.size(); i++)
	{
		rectangle(
			image,
			Point(path[i].x * windowSize - windowSize / 2, path[i].y * windowSize + windowSize / 2),
			Point((path[i].x + 1) * windowSize - windowSize /2, (path[i].y + 1) * windowSize + windowSize / 2),
			Scalar(0, 255, 0), CV_FILLED
		);
	}

	
	// Draw Destination	
	rectangle(
		image,
		Point(destinationPos.x * windowSize - windowSize / 2, destinationPos.y * windowSize + windowSize / 2),
		Point((destinationPos.x + 1) * windowSize - windowSize / 2, (destinationPos.y + 1) * windowSize + windowSize / 2),
		Scalar(0, 255, 255), CV_FILLED
	);

	if(canGo)
	{
		if((float)path[1].y / 7 < 0.25)
		{
			move_msg.relativeX = -0.25;
			move_msg.relativeY = (float)(path[1].x - robotWindowIndex) / 5 * windowSize / 20;
		}
		else
		{
			move_msg.relativeX = (float)path[1].y / 7 * windowSize / 20;
			move_msg.relativeY = -(float)(path[1].x - robotWindowIndex) / 5 * windowSize / 20;
		}

		if(move_msg.relativeX == 0 && move_msg.relativeY == 0)
			move_msg.relativeY = 1;
	}
	else
	{
		move_msg.relativeX = 0.01;
		move_msg.relativeY = 0.4;
	}
	printf("Cognition Node : [%6.3f, %6.3f]\n", move_msg.relativeX, move_msg.relativeY);
}

class masterFollower
{
private:
	clock_t timeStart;
	iCoord masterPos;
	iCoord estimatedPos;
	iCoord estimatedVel;
	iCoord initialTf;

	bool isMsDetected;
	bool isMsLost;

public:
	masterFollower()
	{
		masterPos.x = 0;
		masterPos.y = 0;
		estimatedPos.x = 0;
		estimatedPos.y = 0;
		isMsDetected = false;
	}

	iCoord getMasterPos()
	{
		return masterPos;
	}

	void setMasterPos(iCoord coord)
	{
		masterPos = coord;
	}

	iCoord getEstimatedPos()
	{
		return estimatedPos;
	}
	
	void setEstimatedPos(iCoord coord)
	{
		estimatedPos = coord;
	}

	iCoord getEstimatedVel()
	{
		return estimatedVel;
	}

	void setEstimatedVel(iCoord coord)
	{
		estimatedVel = coord;
	}

	iCoord getInitialTransformation()
	{
		return initialTf;
	}

	void setInitialTransformation(iCoord coord)
	{
		initialTf = coord;
	}

	bool isMasterDetected()
	{
		return isMsDetected;
	}
	
	void setMasterDetected(bool detected)
	{
		isMsDetected = detected;
	}

	bool isMasterLost()
	{
		return isMsLost;
	}

	void setMasterLost(bool lost)
	{
		isMsLost = lost;
	}

	double getTimerDuration()
	{
		return (clock() - timeStart) / (double) CLOCKS_PER_SEC;
	}

	void startTimer()
	{
		timeStart = clock();
	}

	void resetTimer()
	{
		timeStart = 0;
	}
};


class cognitionSubscriber
{
private:
	ros::NodeHandle _nh;
	ros::Subscriber _subLaserScan;
	ros::Subscriber _subMasterPosition;
	ros::Subscriber _subEstimatedMasterPosition;
	obstacleDetector* detector;
	masterFollower* follower;

public:
	cognitionSubscriber()
	{
		_subLaserScan = _nh.subscribe(
			"/laser_scan", 100, &cognitionSubscriber::getLaserScanData, this
		);
		_subMasterPosition = _nh.subscribe(
			"/master", 100, &cognitionSubscriber::getMasterPosition, this
		);
		_subEstimatedMasterPosition = _nh.subscribe(
			"/estimation", 100, &cognitionSubscriber::getEstimatedMasterPosition, this
		);
		detector = new obstacleDetector();
		follower = new masterFollower();
	}

	void getLaserScanData(const no_look_pass_robot::laserScan& laserScanData);
	void getMasterPosition(const no_look_pass_robot::master& masterPosition);
	void getEstimatedMasterPosition(const no_look_pass_robot::estimation& estimatedMasterPosition);
	void odomMsgCallback(const nav_msgs::Odometry& msg);
};

void cognitionSubscriber::getLaserScanData(const no_look_pass_robot::laserScan& laserScanData)
{
	bool testing = true;
	detector->clearImage();
	detector->clearWindows();
	detector->makeWindows(
		laserScanData,
		laserScanData.rangeMax
	);
	detector->drawWindows();
	
	if(testing)
		detector->avoidObstacles(
			0,
			13
		);

	if(!testing)
	{
		// Master Lost start
		if (follower->isMasterDetected() && !follower->isMasterLost() && follower->getMasterPos().x == 0 && follower->getMasterPos().y == 0)
		{
			follower->setMasterLost(true);
			// Start clock
			follower->startTimer();
			// Set current position
			iCoord initialTf;
			initialTf.x = g_odom.pose.pose.position.x;
			initialTf.y = g_odom.pose.pose.position.y;
			follower->setInitialTransformation(initialTf);		
			// Set initial estimated position
			iCoord estimatedTf;
			iCoord convertedEstimationPos;
			iCoord convertedEstimationVel;
			double dAngleTurned = atan2(
				(2 * g_odom.pose.pose.orientation.z * g_odom.pose.pose.orientation.w),
				(1 - (2 * (g_odom.pose.pose.orientation.z * g_odom.pose.pose.orientation.z)))
			);
			convertedEstimationPos.x =
				follower->getEstimatedPos().x * cos(-dAngleTurned) - follower->getEstimatedPos().y * sin(-dAngleTurned);
			convertedEstimationPos.y =
				follower->getEstimatedPos().x * sin(-dAngleTurned) + follower->getEstimatedPos().y * cos(-dAngleTurned);

			estimatedTf.x = initialTf.x + convertedEstimationPos.x;
			estimatedTf.y = initialTf.y + convertedEstimationPos.y;
			follower->setEstimatedPos(estimatedTf);

			convertedEstimationVel.x =
				follower->getEstimatedVel().x * cos(-dAngleTurned) - follower->getEstimatedVel().y * sin(-dAngleTurned);
			convertedEstimationPos.y =
				follower->getEstimatedVel().x * sin(-dAngleTurned) + follower->getEstimatedVel().y * cos(-dAngleTurned);

			follower->setEstimatedVel(convertedEstimationVel);
		}
		else if (follower->isMasterDetected() && follower->isMasterLost())
		{
			// Retreive Master
			if (follower->getMasterPos().x != 0 || follower->getMasterPos().y != 0)
			{
				follower->resetTimer();
				follower->setMasterLost(false);
			}
			// Detect Master Fail
			else if (follower->getTimerDuration() > 4)
			{
				follower->resetTimer();
				follower->setMasterDetected(false);
			}
		}


		// When detect master
		if (follower->isMasterDetected() && !follower->isMasterLost())
		{
			detector->avoidObstacles(
				follower->getMasterPos().x / windowSize,
				follower->getMasterPos().y / windowSize
			);
		}
		// When lost master
		else if(follower->isMasterDetected() && follower->isMasterLost())
		{
			// Should go to estimated position
			detector->avoidObstacles(
				(follower->getEstimatedPos().x - g_odom.pose.pose.position.x) / windowSize,
				(follower->getEstimatedPos().y - g_odom.pose.pose.position.y) / windowSize
			);

			iCoord _nextEstimatedPos;
			_nextEstimatedPos.x = follower->getEstimatedPos().x + follower->getEstimatedVel().x * 0.333f;
			_nextEstimatedPos.y = follower->getEstimatedPos().y + follower->getEstimatedVel().y * 0.333f;
			follower->setEstimatedPos(_nextEstimatedPos);
		}
		// When detect master fail
		else if(!follower->isMasterDetected())
		{
			move_msg.relativeX = 0;
			move_msg.relativeY = 0;
		}



		// Remove past master position
		iCoord _setZero;
		_setZero.x = 0; _setZero.y = 0;
		follower->setMasterPos(_setZero);
	}

	detector->showImage();
}

void cognitionSubscriber::getMasterPosition(const no_look_pass_robot::master& masterPosition)
{
	iCoord _masterPos;
	_masterPos.x = (int)(masterPosition.x * 100); // Because meter
	_masterPos.y = (int)(masterPosition.y * 100); // Because meter
	follower->setMasterPos(_masterPos);
	printf("Master Position ! %f, %f\n", masterPosition.x, masterPosition.y);
	
	if (!follower->isMasterDetected() && !(_masterPos.x == 0 && _masterPos.y == 0))
		follower->setMasterDetected(true);
}


void cognitionSubscriber::getEstimatedMasterPosition(const no_look_pass_robot::estimation& estimatedMasterPosition)
{
	if (!(estimatedMasterPosition.x == 0 && estimatedMasterPosition.y == 0 &&
		estimatedMasterPosition.x_vel == 0 && estimatedMasterPosition.y_vel == 0))
	{
		iCoord _estimatedPos;
		_estimatedPos.x = (int)(estimatedMasterPosition.x * 100);
		_estimatedPos.y = (int)(estimatedMasterPosition.y * 100);

		iCoord _estimatedVel;
		_estimatedVel.x = (int)(estimatedMasterPosition.x_vel * 100);
		_estimatedVel.y = (int)(estimatedMasterPosition.y_vel * 100);

		follower->setEstimatedPos(_estimatedPos);
		follower->setEstimatedVel(_estimatedVel);
	}
}


// Get Odometry Callback function
void cognitionSubscriber::odomMsgCallback(const nav_msgs::Odometry &msg)
{
	// receive a '/odom' message with the mutex
	mutex.lock(); {
		g_odom = msg;
	} mutex.unlock();
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
	ros::Rate rate(3);

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