#ifndef _PATHFINDER_H
#define _PATHFINDER_H
#include <queue>

void FindMazeSolution(int car_xpos, int car_ypos,int map[15][19], int food_list[][2], int rowOfFoodList, std::queue<char> &instructions);
extern std::queue<char> instructions;

#endif
