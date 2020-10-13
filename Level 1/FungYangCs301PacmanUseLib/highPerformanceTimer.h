//======================================================================
//Author: 
//Mr. Fung Yang
//Technician Engineer Research and Design,
//Robotics and control system signal processing Labs,
//Department of Electrical, Computer and Software Engineering,
//The university of Auckland.
//
//Written for teaching design course Compsys301 in ECSE department UOA.
//Date 2012~2020
//=======================================================================

#ifndef highPerformanceTimer_h
#define highPerformanceTimer_h 

#include <windows.h>

//resetTimer(): reset the time counter to zero.
//getTimer(): get the time from reset to now. unit:seconds.
class highPerformanceTimer{
public:
void resetTimer();
double getTimer();

private:	
LARGE_INTEGER freq;
LARGE_INTEGER time1, time2;

};



#endif