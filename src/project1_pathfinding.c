/*
 ============================================================================
 Name        : path_finder.c
 Author      : 
 Version     :
 Copyright   : Your copyright notice
 Description : Hello World in C, Ansi-style
 ============================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <math.h>

#define MAX_OBSTACLES 25 /* maximum number of obstacles */

#define MAX_X 10 /* maximum number of tiles on x axis */
#define MAX_Y 16 /* maximum number of tiles on y axis */
#define MAX_GOALS 5
int world[MAX_X-1][MAX_Y-1];

int num_obstacles = 13; /* number of obstacles */
double obstacle[MAX_OBSTACLES][2] = /* obstacle locations */
{{0.61, 2.743},{0.915, 2.743},{1.219, 2.743},{1.829, 1.219},
{1.829, 1.524},{ 1.829, 1.829}, {1.829, 2.134},{2.743, 0.305},
{2.743, 0.61},{2.743, 0.915},{2.743, 2.743},{3.048, 2.743},
{3.353, 2.743},
{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
{-1,-1},{-1,-1},{-1,-1}};

double start[2] = {0.305, 1.219}; /* start location */
double goal[2] = {3.658, 1.829}; /* goal location */
int action_array[(MAX_X-1) * (MAX_Y-1)];
int action_array_size = 0;
int goals[MAX_GOALS][2];

// move the origin of the map from bottom left to top left
double start_new[2];
int start_pos[2];
int current_pos[2];

int get_min_manhattan_dist(int pos_x, int pos_y);
int manhattan_distance(int x1, int x2, int y1, int y2);
int min_element(int* dist_array, int size);
void print_array(int array[], int length);
void print_2d_array(int array[][MAX_Y], int rows, int columns);
void print_world();
void create_action_array();
int compareArray(int* a,int* b,int size);


void pathfinder(){

	// fill obstacle cells with INF value
	for (int i = 0; i < num_obstacles; i++){
		world[(int)(obstacle[i][0] / 0.305) - 1][(int)(obstacle[i][1] / 0.305) - 1] = INT_MAX;
	}
	// calculate the Manhattan Distance for the free cells
	for (int row=0; row < MAX_X-1; row++){
		for (int col=0; col < MAX_Y-1; col++){
			if (world[row][col] != INT_MAX){
				int dist = get_min_manhattan_dist(row, col);
				world[row][col] = dist;
			}
		}

	}
	print_world();
	// calculate the action order
	create_action_array();
	printf("Order of Actions: \n");
	for (int i = 0; i < action_array_size; i++){
		if (action_array[i] == 0)
			printf("UP\n");
		else if (action_array[i] == 1)
				printf("DOWN\n");
		else if (action_array[i] == 2)
				printf("LEFT\n");
		else if (action_array[i] == 3)
				printf("RIGHT\n");

	}

}
void create_action_array(){
	current_pos[0] = start_pos[0];
	current_pos[1] = start_pos[1];
	int neighbors[4];
	int action = 0;

	while (1){

		if (current_pos[0] == 0){
			neighbors[0] = INT_MAX;
			neighbors[1] = world[ current_pos[0] + 1] [ current_pos[ 1]];
			neighbors[2] = world[ current_pos[0]] [ current_pos[ 1] - 1];
			neighbors[3] =  world[ current_pos[0]] [ current_pos[ 1] + 1];
		}
		else if (current_pos[0] == MAX_X-1){
			neighbors[0] =  world[ current_pos[0] -1] [ current_pos[ 1]];
			neighbors[1] = INT_MAX;
			neighbors[2] = world[ current_pos[0]] [ current_pos[ 1] - 1];
			neighbors[3] =  world[ current_pos[0]] [ current_pos[ 1] + 1];
		}
		else if (current_pos[1] == 0){
			neighbors[0] =  world[ current_pos[0] -1] [ current_pos[ 1]];
			neighbors[1] = world[ current_pos[0] + 1] [ current_pos[ 1]];
			neighbors[2] = INT_MAX;
			neighbors[3] =  world[ current_pos[0]] [ current_pos[ 1] + 1];
		}
		else if(current_pos[1] == MAX_Y-1){
			neighbors[0] =  world[ current_pos[0] -1] [ current_pos[ 1]];
			neighbors[1] = world[ current_pos[0] + 1] [ current_pos[ 1]];
			neighbors[2] = world[ current_pos[0]] [ current_pos[ 1] - 1];
			neighbors[3] = INT_MAX;
		}
		else{
			neighbors[0] =  world[ current_pos[0] -1] [ current_pos[ 1]];
			neighbors[1] = world[ current_pos[0] + 1] [ current_pos[ 1]];
			neighbors[2] = world[ current_pos[0]] [ current_pos[ 1] - 1];
			neighbors[3] =  world[ current_pos[0]] [ current_pos[ 1] + 1];
		}
		int min;
		//give priority to the previous action
		min = neighbors[action];

		for (int i = 0; i < 4; i++){
			if (neighbors[i] < min){

				min = neighbors[i];
				action = i;
			}
		}
		action_array[action_array_size] = action;
		action_array_size++;
		// update current position
		if (action == 0)// go up
			current_pos[0] = current_pos[0] - 1;
		else if (action == 1)// go down
			current_pos[0] = current_pos[0] + 1;
		else if (action == 2)// go left
			current_pos[1] = current_pos[1] - 1;
		else //go right
			current_pos[1] = current_pos[1] + 1;
		// check if goal reached
		int goalReached = 0;
		for (int i = 0; i < MAX_GOALS; i++){
			if (current_pos[1]== goals[i][0] && current_pos[0]== goals[i][1]){
				goalReached = 1;
				break;
			}
		}
		if (goalReached)
			break;


	}
}

int get_min_manhattan_dist(int pos_x, int pos_y){
	int dist_array[MAX_GOALS];
	for (int row = 0; row <= MAX_GOALS; row++){
		dist_array[row] = manhattan_distance(pos_y, goals[row][0], pos_x, goals[row][1]);
	}
//	print_array(dist_array, MAX_GOALS);

	return min_element(dist_array, 5);
}

int manhattan_distance(int x1, int x2, int y1, int y2)
{
  int distance;
  int x_dif, y_dif;

  x_dif = x2 - x1;
  y_dif = y2 - y1;
  if(x_dif < 0)
    x_dif = -x_dif;
  if(y_dif < 0)
    y_dif = -y_dif;
  distance = x_dif + y_dif;
  return distance;
}

int min_element(int* dist_array, int size){
    int min = dist_array[0];
	for (int i = 1; i < size; ++i)
	{
		if (dist_array[i] < min)
		{
			min = dist_array[i];
		}
	}
	return min;
}
void print_array(int array[], int length){
    for (int i = 0; i < length; i++) {
 		printf("%d\t", array[i]);
    }
    printf("\n");
}
void print_2d_array(int array[][MAX_Y], int rows, int columns){
    for (int i = 0; i < rows; i++) {
    	for (int l = 0; l < columns; l++) {
    		if (array[i][l] == INT_MAX)
    			printf("INF\t");
    		else if (start_pos[0] == i && start_pos[1] == l)
    			printf("HERE\t");
    		else
    			printf("%d\t", array[i][l]);
    	}
    	printf("\n");
    }
}
int compareArray(int* a,int* b,int size)	{
	int i;
	for(i=0;i<size;i++){
		if(a[i]!=b[i])
			return 1;
	}
	return 0;
}
void print_world(){
    for (int i = 0; i < MAX_X-1; i++) {
    	for (int l = 0; l < MAX_Y-1; l++) {
    		if (world[i][l] == INT_MAX)
    			printf("INF\t");
    		else if (start_pos[0] == i && start_pos[1] == l)
    			printf("HERE\t");
    		else
    			printf("%d\t", world[i][l]);
    	}
    	printf("\n");
    }
}

int main(void) {
	start_new[0] = start[0];
	start_new[1] = 3.05 - start[1];
	//world initialization
	for (int row=0; row < MAX_X-1; row++){
			for (int col=0; col < MAX_Y-1; col++){
				world[row][col] = -1;
			}

	}

	// translate coordinates to cell in the world
	start_pos[0] = ceil(start_new[0] / 0.305)  - 1;
	start_pos[1] = ceil(start_new[1] / 0.305)- 1;

	// the four different goal positions
	goals[0][0] = ceil((goal[0] / 0.305))  - 1;
	goals[0][1] = ceil((goal[1] / 0.305))  - 1;
	goals[1][0] = ceil((goal[0] / 0.305))  - 1;
	goals[1][1]	= ceil((goal[1] / 0.305)) - 2;
	goals[2][0] = ceil((goal[0] / 0.305)) - 2;
	goals[2][1] = ceil((goal[1] / 0.305)) - 1;
	goals[3][0] = ceil((goal[0] / 0.305));
	goals[3][1] = ceil((goal[1] / 0.305)) - 1;
	goals[4][0] = ceil((goal[0] / 0.305)) - 1;
	goals[4][1] = ceil((goal[1] / 0.305));
	pathfinder();

	return 0;
}














