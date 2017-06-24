#include <ros/ros.h>
#include <no_look_pass_robot/move.h>
#include <no_look_pass_robot/laserScan.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <iomanip>

#define INF 9999
#define WANT_X true
#define WANT_Y false

using namespace cv;

enum windowState {FREE, BLOCK, VISIT};
enum coord {X, Y};

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

/*	for(int i = 0; i < edgeWindows.size(); i++)
	{
			rectangle(
				image,
				Point(edgeWindows[i].x * windowSize, edgeWindows[i].y * windowSize),
				Point((edgeWindows[i].x + 1) * windowSize, (edgeWindows[i].y + 1) * windowSize),
				Scalar(255, 0, 0), CV_FILLED
			);
	}*/
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
	for(int j = -nRobotWindows / 2 + 1;  j <= nRobotWindows / 2; j++)
	{
		for(int k = -nRobotWindows / 2; k < nRobotWindows / 2; k++)
		{
			if(xIndex + k >= 0 && xIndex + k < windowWidth && yIndex >= 0 && yIndex < windowHeight)
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
	windows[destinationPos.x][destinationPos.y] = FREE;
	windows[destinationPos.x - 1][destinationPos.y] = FREE;
	windows[destinationPos.x][destinationPos.y + 1] = FREE;
	windows[destinationPos.x - 1][destinationPos.y + 1] = FREE;
	printf("des X, Y : %d, %d\n", destinationPos.x, destinationPos.y);
	
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
			printf("Sort complete!\n");
			
			// 가장 가까운 edgeWindow 부터 탐색
			iCoord checkCollision;
			int lookSideMore;
			int lookBottomMore;
			int times = 0;
			canGo = false;
			for(int k = 0; k < 15; k++)
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
					edgePos.x = edgeWindows[sortedEdgesIndex[j]].x + leftOrRight * times;
					edgePos.y = edgeWindows[sortedEdgesIndex[j]].y - lookBottomMore * times;
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
				if(canGo)
					break;
			}

			// 갈곳이 없을 경우
			if(!canGo)
			{
				iCoord backward;
				backward.x = robotWindowIndex + 1;
				backward.y = 0;
				path.push_back(backward);
				break;
			}
		}
		else // 충돌 X
		{
			path.push_back(destinationPos);
			break;
		}
	}

	// 필요 없는 길 삭제
	if(canGo)
	{
		iCoord currentPos;
		iCoord nextPos;
		iCoord detectedPos;
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
	else
	{
		while(path.size() > 2)
			path.erase(path.begin() + 1);
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

	if((float)path[1].y / 6 < 0.2)
	{
		move_msg.relativeX = -0.2;
		move_msg.relativeY = (float)(path[1].x - robotWindowIndex) / 6;
	}
	else
	{
		move_msg.relativeX = (float)path[1].y / 6;
		move_msg.relativeY = -(float)(path[1].x - robotWindowIndex) / 6;
	}

	if(move_msg.relativeX == 0 && move_msg.relativeY == 0)
		move_msg.relativeY = 1;

	printf("Cognition Node : [%6.3f, %6.3f]\n", move_msg.relativeX, move_msg.relativeY);
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
	//detector->drawWindows();
	detector->avoidObstacles(0, 12);
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
