//======================================================================
//Author: 
//Mr. Fung Yang
//Senior Technician Engineer Research and Design,
//Robotics and Control system signal processing Labs,
//Department of Electrical, Computer and Software Engineering,
//The University of Auckland.
//
//Written for teaching design course Compsys301 in ECSE department of UOA.
//
//This example program uses the pacman robot simulation library written by Mr. Fung Yang.
//
//Date 2012~2020
//=======================================================================

#include "mainFungGLAppEngin.h" //a must
#include "data.h" //a must
#include "highPerformanceTimer.h"//just to include if timer function is required by user.
#include <vector>
#include <iostream>
#include <queue>
#include <cmath>
#include "pathFinder.h"

using namespace std;
queue<char> instructions;


//{=================================================
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//these global variables must be defined here with no modification.
float virtualCarLinearSpeed;//can get and set
float virtualCarAngularSpeed;//can get and set
float currentCarAngle;//can get and set
float currentCarPosCoord_X, currentCarPosCoord_Y;//can get and set

int sensorPopulationAlgorithmID;//can set
float sensorSeparation;//can set
float num_sensors;//can set

vector<int> virtualCarSensorStates; //can get

vector<ghostInfoPack> ghostInfoPackList;// can get
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//}=================================================

highPerformanceTimer myTimer;
highPerformanceTimer myUturnTimer;
highPerformanceTimer LevelTimer;
static int currentCell[2];
static bool UturnFlag = false;
static bool blackSensorFlag = false;

//just a helper function
void setVirtualCarSpeed(float linearSpeed, float angularSpeed)
{
	virtualCarLinearSpeed = linearSpeed;
	virtualCarAngularSpeed = angularSpeed;
}

//The Only TWO functions Students need to modify to add their own sensor guided
//path following control and Map path search calculations.
//{=================================================
float virtualCarLinearSpeed_seed;
float virtualCarAngularSpeed_seed;

void CoToCe(float Coo_X, float Coo_Y, int& cellX, int& cellY)
{
	int i = 0;
	int j = 0;
	float x = -7.0;
	float y = 5.2448;
	while (true)
	{
		x += 0.7242284;
		if (Coo_X <= x) break;
		++i;
	}

	while (true)
	{
		y -= 0.6974309;
		if (Coo_Y > y) break;
		++j;
	}
	cellX = i;
	cellY = j;
}

int virtualCarInit()
{
	//sensorPopulationAlgorithmID = PLACE_SENSORS_AUTO_SEP;
	sensorPopulationAlgorithmID = PLACE_SENSORS_SEP_USER_DEFINED;
	num_sensors = 7;
	sensorSeparation = 0.08;

	virtualCarLinearSpeed_seed = 0.6;
	virtualCarAngularSpeed_seed = 35;

	currentCarPosCoord_X = cellToCoordX(9);
	currentCarPosCoord_Y = cellToCoordY(13);
	currentCarAngle = 90;

	LevelTimer.resetTimer();

	CoToCe(currentCarPosCoord_X, currentCarPosCoord_Y, currentCell[0], currentCell[1]);

	//Level 1 Food list update

	int food_count = 0;
	int food_list[15 * 19][2] = { 0 };

	for (int x = 0; x < 15; ++x)
	{
		for (int y = 0; y < 19; ++y)
		{
			if (map[x][y] == 0)
			{
				food_list[food_count][0] = x;
				food_list[food_count][1] = y;
				food_count++;
			}
		}
	}
	
	FindMazeSolution(currentCell[1], currentCell[0], map, food_list, food_count, instructions);

	return 1;
}

char instructionParser(queue<char>& instructions)
{
	// Up = [45, 135), Left = [135, 225), Down = [225, 315), Right = [315, 45)
	if (!instructions.empty())
	{
		if ((instructions.front() == 'U' && (currentCarAngle >= 045 && currentCarAngle < 135)) ||
			(instructions.front() == 'L' && (currentCarAngle >= 135 && currentCarAngle < 225)) ||
			(instructions.front() == 'D' && (currentCarAngle >= 225 && currentCarAngle < 315)) ||
			(instructions.front() == 'R' && (currentCarAngle >= 315 || currentCarAngle < 045))) return 'S'; // Go Straight
		if ((instructions.front() == 'U' && (currentCarAngle >= 315 || currentCarAngle < 045)) ||
			(instructions.front() == 'L' && (currentCarAngle >= 045 && currentCarAngle < 135)) ||
			(instructions.front() == 'D' && (currentCarAngle >= 135 && currentCarAngle < 225)) ||
			(instructions.front() == 'R' && (currentCarAngle >= 225 && currentCarAngle < 315))) return 'L'; // Turn Left
		if ((instructions.front() == 'U' && (currentCarAngle >= 135 && currentCarAngle < 225)) ||
			(instructions.front() == 'L' && (currentCarAngle >= 225 && currentCarAngle < 315)) ||
			(instructions.front() == 'D' && (currentCarAngle >= 315 || currentCarAngle < 045)) ||
			(instructions.front() == 'R' && (currentCarAngle >= 045 && currentCarAngle < 135))) return 'R'; // Turn Right
		return 'U'; // U turn
	}
	return 'P'; // Parking(Stop)
}

int virtualCarUpdate()
{
	//{----------------------------------
	//process sensor state information
	float halfTiltRange = (num_sensors - 1.0) / 2.0;
	float tiltSum = 0.0;
	float blackSensorCount = 0.0;
	for (int i = 0; i < num_sensors; i++)
	{
		if (virtualCarSensorStates[i] == 0)
		{
			float tilt = (float)i - halfTiltRange;
			tiltSum += tilt;
			blackSensorCount += 1.0;
		}
	}
	//}------------------------------------
	int nowCell[2];

	CoToCe(currentCarPosCoord_X, currentCarPosCoord_Y, nowCell[0], nowCell[1]);
	if (UturnFlag && myUturnTimer.getTimer() < 1.0) return 1;
	UturnFlag = false;
	char CMD = instructionParser(instructions); // read and parse the instruction
	if (blackSensorFlag)
	{
		if (CMD == 'R') setVirtualCarSpeed(0.0, -virtualCarAngularSpeed_seed);
		else if (CMD == 'L') setVirtualCarSpeed(0.0, virtualCarAngularSpeed_seed);
		else if (CMD == 'S')
		{
			if (instructions.front() == 'U')
			{
				if (currentCarAngle > 90) setVirtualCarSpeed(0.0, -virtualCarAngularSpeed_seed * 1.75); //turn right
				else setVirtualCarSpeed(0.0, virtualCarAngularSpeed_seed * 1.75);
			}
			else if (instructions.front() == 'L')
			{
				if (currentCarAngle > 180) setVirtualCarSpeed(0.0, -virtualCarAngularSpeed_seed * 1.75); //turn right
				else setVirtualCarSpeed(0.0, virtualCarAngularSpeed_seed * 1.75);
			}
			else if (instructions.front() == 'D')
			{
				if (currentCarAngle > 270) setVirtualCarSpeed(0.0, -virtualCarAngularSpeed_seed * 1.75); //turn right
				else setVirtualCarSpeed(0.0, virtualCarAngularSpeed_seed * 1.75);
			}
			else if (instructions.front() == 'R')
			{
				if (currentCarAngle > 0) setVirtualCarSpeed(0.0, -virtualCarAngularSpeed_seed * 1.75); //turn right
				else setVirtualCarSpeed(0.0, virtualCarAngularSpeed_seed * 1.75);
			}
		}
		else
		{
			setVirtualCarSpeed(0.0, virtualCarAngularSpeed_seed * 5.25);
			myUturnTimer.resetTimer();
			UturnFlag = true;
			return 1;
		}
	}
	if (CMD == 'P') setVirtualCarSpeed(0.0, 0.0); // Parking
	//else if the vehicle is in the same cell, continue execuing the current CMD and start doing posture adjustment
	else if ((currentCell[0] == nowCell[0]) && (currentCell[1] == nowCell[1]))
	{
		if (blackSensorCount > static_cast<float>(0.0))
		{
			blackSensorFlag = false;
			if (CMD == 'S')
			{
				if (abs(tiltSum) >= 5) tiltSum /= 4.5;
                else if (abs(tiltSum) >= 3 && virtualCarSensorStates[0] != 0 && virtualCarSensorStates[6] != 0) tiltSum /= 2;
                setVirtualCarSpeed(virtualCarLinearSpeed_seed * 1.5, virtualCarAngularSpeed_seed * tiltSum);
                if ((currentCarAngle < 030 || currentCarAngle > 330) ||
                    (currentCarAngle < 120 && currentCarAngle > 060) ||
                    (currentCarAngle < 210 && currentCarAngle > 150) ||
                    (currentCarAngle < 300 && currentCarAngle > 240)) setVirtualCarSpeed(virtualCarLinearSpeed_seed * 2, virtualCarAngularSpeed_seed * tiltSum);
			}
			else if (CMD == 'L')
			{
				if (virtualCarSensorStates[0] == 0 || (virtualCarSensorStates[1] == 0 && virtualCarSensorStates[2] == 0))
				{
					setVirtualCarSpeed(0.15, virtualCarAngularSpeed_seed * 2);
				}
				else setVirtualCarSpeed(0.3, 60);
			}
			else if (CMD == 'R')
			{
				if ((virtualCarSensorStates[4] == 0 && virtualCarSensorStates[5] == 0) || virtualCarSensorStates[6] == 0)
				{
					setVirtualCarSpeed(0.15, -virtualCarAngularSpeed_seed * 2);
				}
				else setVirtualCarSpeed(0.3, -60);
			}
			else if (CMD == 'U')
			{
				setVirtualCarSpeed(0.0, virtualCarAngularSpeed_seed * 5.25);
				myUturnTimer.resetTimer();
				UturnFlag = true;
			}
		}
		else blackSensorFlag = true;
	}
	else // not the same cell, stop the vehicle and pop the current instruction
	{
		cout << "=====================================" << endl;
		cout << "Current INS, CMD: " << instructions.front() << ", " << CMD << "\n";
		cout << "current car X, Y, theta = " << currentCarPosCoord_X << " , " << currentCarPosCoord_Y << " , " << currentCarAngle << endl;
		cout << "Last     Cell X, Y = " << currentCell[0] << " , " << currentCell[0] << endl;
		cout << "Computed Cell X, Y = " << nowCell[0] << " , " << nowCell[1] << endl;
		cout << "API      Cell X, Y = " << coordToCellX(currentCarPosCoord_X) << " , " << coordToCellY(currentCarPosCoord_Y) << endl;
		cout << "\n=====================================\n";
		cout << "Run Time:  " << LevelTimer.getTimer() << " seconds\n";
		instructions.pop();
		setVirtualCarSpeed(0.0, 0.0);
	}
	// Update the current cell
	currentCell[0] = nowCell[0];
	currentCell[1] = nowCell[1];

	//below is optional. just to provid some status report and function test result .
	//You can try to use "printf()" to reimplemet this "cout" c++ section in a c style instead.
	//{--------------------------------------------------------------	
	if (myTimer.getTimer() > 999)
	{
		myTimer.resetTimer();

		cout << "=====================================" << endl;
		cout << "current car X, Y, theta = " << currentCarPosCoord_X << " , " << currentCarPosCoord_Y << " , " << currentCarAngle << endl;
		cout << "Computed Cell X, Y = " << nowCell[0] << " , " << nowCell[1] << endl;
		cout << "API      Cell X, Y = " << coordToCellX(currentCarPosCoord_X) << " , " << coordToCellY(currentCarPosCoord_Y) << endl;
		/*
		cout << "-----------------------------------------" << endl;
		cout << " ghost list info:" << endl;
		for (int i = 0; i < ghostInfoPackList.size(); i++)
		{
			cout << "g[" << i << "]: (" << ghostInfoPackList[i].coord_x << ", " << ghostInfoPackList[i].coord_y << "); [s=" <<
				ghostInfoPackList[i].speed << "; [d=" << ghostInfoPackList[i].direction << "]; [T=" << ghostInfoPackList[i].ghostType << "]" << endl;
		}
		cout << "-----------------------------------------" << endl;
		int randNumber = rand_nextInt(10);
		cout << " a rand number between 0 ~ 10 = " << randNumber << endl;
		randNumber = rand_nextInt(10, 20);
		cout << " a rand number between 10 ~ 20 = " << randNumber << endl;
		cout << "-----------------------------------------" << endl;
		cout << "map[1][9] = " << map[1][9] << endl;
		cout << "food_list[1][1] = " << food_list[1][1] << endl;
		*/
	}
	//}---------------------------------------------------------------
	return 1;
}
//}=============================================================

int main(int argc, char** argv)
{
	FungGlAppMainFuction(argc, argv);
	
	return 0;
}
