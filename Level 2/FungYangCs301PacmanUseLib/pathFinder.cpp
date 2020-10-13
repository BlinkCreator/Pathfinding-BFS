#include <iostream>
#include <stdlib.h>
#include <queue>
#include <list>
#include <stack>

using namespace std;

static int vmap[15][19];
//map length and height
static int ymap = 19;
static int xmap = 15;
//position in map

static int food_xpos[285];
static int food_ypos[285];

static int car_Xpos;
static int car_Ypos;

//queue of x and y position
static queue<int> xq;
static queue<int> yq;

//queue of all co-ordinates in stack form first in last out
static stack<int> directions_x;
static stack<int> directions_y;

//Stack of instructions back to front ++++++++
static stack<char> directions;

//direction for x(up, down) and y(left, right)
static int dx[4] = { -1, 1, 0, 0 };
static int dy[4] = { 0, 0, 1,-1 };

static int nodes_in_current_layer;
static int nodes_in_next_layer;

//Lists
extern queue<char> instructions;


// Heuristic functino for seek_nodes is called when two
// neighboring nodes have food. Finds the food with the 
// shortest path to the wall
int dfs(int x, int y, int xnode, int ynode, int dir) {
	int count = 0;
	int xnod = xnode;
	int ynod = ynode;
	while ((vmap[xnod][ynod] != 1) && (vmap[xnod][ynod] != 2) && (xnod <= xmap) && (ynod <= ymap)) {
		if (ynod <= 0 or xnod <= 0) {
			break;
		}
		if (xnod >= xmap or ynod >= ymap) {
			break;
		}
		ynod = ynod + dy[dir];
		xnod = xnod + dx[dir];
		count++;
	}
	return count;
}

/*
	Helper function for BFS takes the
	x and y position of current node
	and finds viable neighboring nodes to add to queue.
*/
static void Seek_nodes(int x, int y) {
	//sets node position to x and y values
	int ynode;
	int xnode;
	int y2node;
	int x2node;

	/*
	 first for loops looks for viable intersections where there
	 are two foods next to current node. Follows each path until
	 it reaches a wall and tallies a count. Compares the count
	 and takes the path with the smallest count.

	*/
	//cycles through all the directions
	for (int i = 0; i < 4; i++)
	{
		//cycles through all the directions
		for (int j = 0; j < 4; j++) {
			if (i != j) {
				ynode = y + dy[i];
				xnode = x + dx[i];
				y2node = y + dy[j];
				x2node = x + dx[j];
				if (ynode <= 0 or xnode <= 0 or y2node <= 0 or x2node <= 0) {
					continue;
				}
				if (xnode >= xmap or ynode >= ymap or x2node >= xmap or y2node >= ymap) {
					continue;
				}
				if (vmap[xnode][ynode] == 1 || vmap[xnode][ynode] == 2 or vmap[x2node][y2node] == 1 || vmap[x2node][y2node] == 2) {
					continue;
				}
				if (vmap[xnode][ynode] == 5 && vmap[x2node][y2node] == 5) {
					int c1 = dfs(x, y, xnode, ynode, i);
					int c2 = dfs(x, y, x2node, y2node, j);
					if (c1 < c2) {
						xq.push(xnode);
						yq.push(ynode);
						directions_x.push(xnode);
						directions_y.push(ynode);

						vmap[xnode][ynode] = 2;
						nodes_in_next_layer += 1;

					}
					else {
						xq.push(x2node);
						yq.push(y2node);
						directions_x.push(x2node);
						directions_y.push(y2node);

						vmap[x2node][y2node] = 2;
						nodes_in_next_layer += 1;
					}


				}
			}
		}
	}
	//cycles through all the directions
	for (int i = 0; i < 4; i++)
	{
		//sets node to any of the one directions
		ynode = y + dy[i];
		xnode = x + dx[i];

		//checks if node is out of bounds
		if (ynode <= 0 or xnode <= 0) {
			continue;
		}
		if (xnode >= xmap or ynode >= ymap) {
			continue;
		}

		// checks if node is a wall or already visited
		if (vmap[xnode][ynode] == 1 || vmap[xnode][ynode] == 2) {
			continue;
		}

		// Node is viable so adds to queue
		xq.push(xnode);
		yq.push(ynode);
		directions_x.push(xnode);
		directions_y.push(ynode);

		// marks node as visited
		vmap[xnode][ynode] = 2;
		nodes_in_next_layer += 1;

	}

}

static void bfs(int car_xpos, int car_ypos, int map[15][19], int food_list[][2], int rowOfFoodList) {
	nodes_in_current_layer = 1;
	nodes_in_next_layer = 0;
	for (int x{}; x < xmap; ++x)
	{
		// check makes correct copy
		for (int y{}; y < ymap; ++y)
		{
			vmap[x][y] = map[x][y];
		}
	}

	// adds food into vmap
	for (int i = 0; i < rowOfFoodList; ++i)
	{
		if (food_ypos[i] != -1 and food_xpos[i] != -1)
		{
			vmap[food_xpos[i]][food_ypos[i]] = 5;
		}
	}

	// Intializes the starting position to find the path
	xq.push(car_xpos);
	yq.push(car_ypos);
	directions_x.push(car_xpos);
	directions_y.push(car_ypos);
	vmap[car_xpos][car_ypos] = 2;

	int x;
	int y;
	int check = 0;

	// While the que is not empty check to see if current position is at food
	while (xq.size() > 0)
	{
		// Sets current pos to the front node then deletes the current node
		x = xq.front();
		xq.pop();
		y = yq.front();
		yq.pop();
		for (int i = 0; i < rowOfFoodList; i++)
		{
			// Checks to see if current node position is at food if so exits function
			// updates carpos to position of the food
			if (x == food_xpos[i] && y == food_ypos[i] && food_xpos[i] != -1 && food_xpos[i] != 0)
			{
				car_Xpos = x;
				car_Ypos = y;
				food_xpos[i] = -1;
				food_ypos[i] = -1;
				check = 1;
				break;
			}
		}

		// If food is found breaks the loop and empties queue
		if (check == 1) {
			while (xq.size() > 0)
			{
				xq.pop();
				yq.pop();
			}
			break;
		}

		// seeks neighboring nodes to current node and pushes to queue
		Seek_nodes(x, y);
		nodes_in_current_layer -= 1;

		// Updates current layer and resets next layer
		// if no nodes in current layer
		if (nodes_in_current_layer == 0)
		{
			nodes_in_current_layer = nodes_in_next_layer;
			nodes_in_next_layer = 0;
		}
	}

	// assigns the direction instructions to the nieghboring node, starting at the back and ending at the front
	//directional instructions must be written backwards
	//sets node position to x and y values
	int ynode;
	int xnode;

	while (directions_x.size() > 0) {
		// x and y are now at the food position
		for (int i = 0; i < 4; i++)
		{
			//sets node to any of the one directions
			ynode = y + dy[i];
			xnode = x + dx[i];

			//checks if node is out of bounds
			if (ynode < 0 or xnode < 0) {
				continue;
			}
			if (xnode >= xmap or ynode >= ymap) {
				continue;
			}
			if (map[xnode][ynode] == 1) {
				continue;
			}
			if (vmap[xnode][ynode] != 2) {
				continue;
			}

			if (xnode == directions_x.top() && ynode == directions_y.top())
			{
				x = directions_x.top();
				y = directions_y.top();

				//marks in the virutal map that it's been visited by the path finding
				vmap[xnode][ynode] = 3;

				if (i == 0) {

					//reverse of U which is D
					directions.push('D');
				}
				if (i == 1) {

					//reverse of D which is U
					directions.push('U');
				}
				if (i == 2) {
					//reverse of R which is L
					directions.push('L');

				}
				if (i == 3) {

					//reverse of L which is R
					directions.push('R');

				}
			}
		}

		//removes the last added element
		//sets x and y to the most recent element
		directions_x.pop();
		directions_y.pop();
	}
}

/*
Solves the maze and updates a queue of instructions that eats all food within an entire maze
takes a,
x position of car within map
y position of car within map
map to solve
list of food
length of food list
queue instructions
returns nothing but updates given queue to all the right directions
*/
void FindMazeSolution(int car_xpos, int car_ypos, int map[15][19], int food_list[][2], int rowOfFoodList, queue<char>& instructions) {

	// intializes arguement of car position so it can be maniputlated
	int carxpos = car_xpos;
	int carypos = car_ypos;

	// breaks down foood list into x and y positions
	for (int i{}; i < rowOfFoodList; ++i) {

		// checks to see if it's food or if there null foods definied in list
		if (food_list[i][0] == 0 && food_list[i][1] == 0) continue;

		food_xpos[i] = food_list[i][0];
		food_ypos[i] = food_list[i][1];
	};

	// creates index of the length of foodlist to cycle through
	int index;

	index = rowOfFoodList;

	/*
	 As long as there is food it will find
	 the shortest path to the nearest food.
	*/
	while (index > 0)
	{
		// marks food in the list as eaten when found 
		// updates directions stack
		bfs(carxpos, carypos, map, food_list, rowOfFoodList);

		//updates car position to the food found.
		carxpos = car_Xpos;
		carypos = car_Ypos;

		// adds directions to the nearest food into a queue 
		// of all instructions that solves the maze.
		while (directions.size() > 0)
		{
			instructions.push(directions.top());
			directions.pop();
		}
		index--;
	}
}