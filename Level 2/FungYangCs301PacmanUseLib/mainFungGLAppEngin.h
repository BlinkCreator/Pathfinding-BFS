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
//Header file uses the pacman robot simulation library written by Mr. Fung Yang.
//
//Date 2012~2020
//=======================================================================

#ifndef _MAIN_FUNG_GL_APP_ENGIN_
#define _MAIN_FUNG_GL_APP_ENGIN_

#define RED_GHOST 0
#define GREEN_GHOST 1
#define BLUE_GHOST 2

#define PLACE_SENSORS_AUTO_SEP 0
#define PLACE_SENSORS_SEP_USER_DEFINED 1


int FungGlAppMainFuction(int argc, char** argv);
int virtualCarInit();
int virtualCarUpdate();

float cellToCoordX(float cell_x);
float cellToCoordY(float cell_y);
int coordToCellX(float coord_x);
int coordToCellY(float coord_y);

class ghostInfoPack
{
public:
	double	coord_x, coord_y;
	int direction;
	float speed;
	int ghostType;
};

//based and link to mazeGen.h
int rand_nextInt(int upperBond);//return rand number from 0 to (upperBond - 1) inclusive.
int rand_nextInt(int lowerBond, int upperBond);//return rand number from lowerBond to (upperBond - 1) inclusive.

#endif
