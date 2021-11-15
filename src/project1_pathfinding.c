/*
 ============================================================================
 Name        : path_motion_ev3_proj1.c
 Authors     : Fotios Lygerakis, Mehul Vishal Sadh
 Version     : 1.0
 Description : Manhattan Field Path Planner and Motion Navigator in C
 ============================================================================
*/

#include <ev3.h>
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <math.h>

#define PI 3.14
#define BLOCK_DISTANCE 30.5 /* cell width */

#define MAX_OBSTACLES 25 /* maximum number of obstacles */

#define MAX_X 16 /* maximum number of tiles on x axis */
#define MAX_Y 10 /* maximum number of tiles on y axis */
#define MAX_GOALS 5
int world[MAX_Y-1][MAX_X-1];

int L_motor = OUT_C;
int R_motor = OUT_A;
int speed = 20;
int rot_speed = 8;
int less_speed = 8;

int num_obstacles = 18; /* number of obstacles */
double obstacle[MAX_OBSTACLES][2] = /* obstacle locations */
{{0.915, 0.305},{0.915, 0.61},{0.915, 0.915},{0.915, 1.22},
{0.915, 1.525},{ 0.915, 1.83}, {1.83, 1.22},{1.83, 1.525},
{1.83, 1.83},{1.83, 2.135},{1.83, 2.44},{1.83, 2.745},
{2.745, 1.525},
{2.745, 1.83},{3.355 ,0.915},{3.355, 1.22},{3.66, 0.915},{3.66, 1.22},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
{-1,-1},{-1,-1},{-1,-1}};

double start[2] = {0.305, 0.61}; /* start location */
double goal[2] = {3.66, 1.83}; /* goal location */
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
void print_world(int pos_x, int pos_y);
void create_action_array();
int compareArray(int* a,int* b,int size);

void move_forward();
void move_left();
void move_right();
void turn_back();

// distance is in centimeters
void move_forward() {

	double Kd = 8;   // Derivative Gain
    ResetRotationCount(OUT_AC);
    int angle_rotated = (int)ceil((180 * BLOCK_DISTANCE / (PI * 2.75)));
    while(1) {
    	OnFwdSync(OUT_AC, Kd * (1 - (MotorRotationCount(R_motor) + MotorRotationCount(L_motor))/(2*angle_rotated)));
    	Wait(10);
    	if((MotorRotationCount(R_motor) + MotorRotationCount(L_motor))/2 == angle_rotated) {
    		break;
    	}
    }

    Off(OUT_AC);

	// compensating the errors
    if(MotorRotationCount(R_motor) < angle_rotated) {
		RotateMotor(R_motor, less_speed, angle_rotated - MotorRotationCount(R_motor));
		Off(OUT_AC);
		Wait(200);
	}

	if(MotorRotationCount(R_motor) > angle_rotated) {
		RotateMotor(R_motor, -less_speed, angle_rotated - MotorRotationCount(R_motor));
		Off(OUT_AC);
		Wait(200);
	}

	if(MotorRotationCount(L_motor) < angle_rotated) {
		RotateMotor(L_motor, less_speed, angle_rotated - MotorRotationCount(L_motor));
		Off(OUT_AC);
		Wait(200);
	}

	if(MotorRotationCount(L_motor) > angle_rotated) {
		RotateMotor(L_motor, -less_speed, angle_rotated - MotorRotationCount(L_motor));
		Off(OUT_AC);
		Wait(200);
	}

}

void move_left() {

	ResetRotationCount(OUT_AC);
	double Kd = 8;
	while((2.75 * (abs(MotorRotationCount(L_motor)) + MotorRotationCount(R_motor))) < 945) {
		OnFwdReg(R_motor, Kd * (1 - MotorRotationCount(R_motor)/170));
		OnRevReg(L_motor, Kd * (1 - MotorRotationCount(L_motor)/170));
		Wait(10);
	}

	Off(OUT_AC);
	Wait(200);

	// compensating the errors in right wheel
	if(((abs(MotorRotationCount(L_motor)) + MotorRotationCount(R_motor)) < 303) || ((abs(MotorRotationCount(L_motor)) + MotorRotationCount(R_motor)) > 303)) {

		if(MotorRotationCount(R_motor) < 151) {
		RotateMotor(R_motor, less_speed, 303 - (MotorRotationCount(R_motor) + abs(MotorRotationCount(L_motor))));
		Off(OUT_AC);
		Wait(200);
		}

		else if(MotorRotationCount(R_motor) > 151) {
			RotateMotor(R_motor, -less_speed, 303 - (MotorRotationCount(R_motor) + abs(MotorRotationCount(L_motor))));
			Off(OUT_AC);
			Wait(200);
		}
	}

	// compensating the errors in left wheel
	if(((abs(MotorRotationCount(L_motor)) + MotorRotationCount(R_motor)) < 303) || ((abs(MotorRotationCount(L_motor)) + MotorRotationCount(R_motor)) > 303)) {

		if(MotorRotationCount(L_motor) > -152) {
			RotateMotor(L_motor, less_speed, 303 - (MotorRotationCount(R_motor) + abs(MotorRotationCount(L_motor))));
			Off(OUT_AC);
			Wait(200);
		}

		else if(MotorRotationCount(L_motor) < -152) {
			RotateMotor(L_motor, -less_speed, 303 - (MotorRotationCount(R_motor) + abs(MotorRotationCount(L_motor))));
			Off(OUT_AC);
			Wait(200);
		}

	}

    move_forward();
    Off(OUT_AC);

}

void move_right() {

	ResetRotationCount(OUT_AC);
	double Kd = 8;
	while((2.75 * (abs(MotorRotationCount(R_motor)) + MotorRotationCount(L_motor))) < 945) {
		OnFwdReg(L_motor, Kd * (1 - MotorRotationCount(L_motor)/170));
		OnRevReg(R_motor, Kd * (1 - MotorRotationCount(R_motor)/170));
		Wait(10);
	}
	Off(OUT_AC);
	Wait(200);

	// compensating the errors in right wheel
	if(((abs(MotorRotationCount(R_motor)) + MotorRotationCount(L_motor)) < 303) || ((abs(MotorRotationCount(R_motor)) + MotorRotationCount(L_motor)) > 303)) {

		if(MotorRotationCount(R_motor) > -152) {
			RotateMotor(R_motor, less_speed, 303 - (MotorRotationCount(L_motor) + abs(MotorRotationCount(R_motor))));
			Off(OUT_AC);
			Wait(200);
		}

		else if(MotorRotationCount(R_motor) < -152) {
			RotateMotor(R_motor, -less_speed, 303 - (MotorRotationCount(L_motor) + abs(MotorRotationCount(R_motor))));
			Off(OUT_AC);
			Wait(200);
		}

	}

	// compensating the errors in left wheel
	if(((abs(MotorRotationCount(R_motor)) + MotorRotationCount(L_motor)) < 303) || ((abs(MotorRotationCount(R_motor)) + MotorRotationCount(L_motor)) > 303)) {

		if(MotorRotationCount(L_motor) < 151) {
		RotateMotor(L_motor, less_speed, 303 - (MotorRotationCount(L_motor) + abs(MotorRotationCount(R_motor))));
		Off(OUT_AC);
		Wait(200);
		}

		else if(MotorRotationCount(L_motor) > 151) {
			RotateMotor(L_motor, -less_speed, 303 - (MotorRotationCount(L_motor) + abs(MotorRotationCount(R_motor))));
			Off(OUT_AC);
			Wait(200);
		}
	}

    move_forward();
    Off(OUT_AC);

}

void turn_back() {

	ResetRotationCount(OUT_AC);
	while((5.6 * (abs(MotorRotationCount(R_motor)) + MotorRotationCount(L_motor))) < 3636) {
		OnFwdReg(L_motor, rot_speed);
		OnRevReg(R_motor, rot_speed);
		Wait(80);
	}
	Off(OUT_AC);
	Wait(200);
	move_forward();
	Off(OUT_AC);

}

void pathfinder(){
	int i, l, row, col;

	// fill obstacle cells with INF value
	for (i = 0; i < num_obstacles; i++){
		world[(int)round(MAX_Y - obstacle[i][1] / 0.305) - 1][(int)round(obstacle[i][0] / 0.305) - 1] = INT_MAX;
	}

	// calculate the Manhattan Distance for the free cells
	for ( row=0; row < MAX_Y-1; row++){
		for ( col=0; col < MAX_X-1; col++) {
			if (world[row][col] != INT_MAX){
				int dist = get_min_manhattan_dist(row, col);
				world[row][col] = dist;
			}
		}

	}

	// calculate the action order
	create_action_array();
	printf("Order of Actions: \n");
	for ( i = 0; i < action_array_size; i++){
		if (action_array[i] == 0)
			printf("Forward Cell\n");
		else if (action_array[i] == 1)
				printf("Left Cell\n");
		else if (action_array[i] == 2)
				printf("Right Cell\n");
		else if (action_array[i] == 3)
						printf("Turn Around\n");


	}

}
void create_action_array(){
	int i, l, row, col;

	//TermPrintln("In pathfinder World Loop");
	current_pos[0] = start_pos[0];
	current_pos[1] = start_pos[1];
	print_world(current_pos[0], current_pos[1]);

	int neighbors[4];
	/* actions can be
		1: move to the cell in front
		2: move to the cell to the right
		3: move in the cell to the left
	*/
	int prev_direction = -1;
	int direction = -1;
	int action = 0;

	while (1){

		if (current_pos[0] == 0){
			neighbors[0] = INT_MAX;
			neighbors[1] =  world[ current_pos[0]] [ current_pos[ 1] + 1];
			neighbors[2] = world[ current_pos[0] + 1] [ current_pos[ 1]];
			neighbors[3] = world[ current_pos[0]] [ current_pos[ 1] - 1];
		}
		else if (current_pos[0] == MAX_Y-1){
			neighbors[0] =  world[ current_pos[0] -1] [ current_pos[ 1]];
			neighbors[1] =  world[ current_pos[0]] [ current_pos[ 1] + 1];
			neighbors[2] = INT_MAX;
			neighbors[3] = world[ current_pos[0]] [ current_pos[ 1] - 1];
		}
		else if (current_pos[1] == 0){
			neighbors[0] =  world[ current_pos[0] -1] [ current_pos[ 1]];
			neighbors[1] =  world[ current_pos[0]] [ current_pos[ 1] + 1];
			neighbors[2] = world[ current_pos[0] + 1] [ current_pos[ 1]];
			neighbors[3] = INT_MAX;
		}
		else if(current_pos[1] == MAX_X-1){
			neighbors[0] =  world[ current_pos[0] -1] [ current_pos[ 1]];
			neighbors[1] = INT_MAX;
			neighbors[2] = world[ current_pos[0] + 1] [ current_pos[ 1]];
			neighbors[3] = world[ current_pos[0]] [ current_pos[ 1] - 1];
		}
		else{
			neighbors[0] =  world[ current_pos[0] -1] [ current_pos[1]];
			neighbors[1] =  world[ current_pos[0]] [ current_pos[1] + 1];
			neighbors[2] = world[ current_pos[0] + 1] [ current_pos[1]];
			neighbors[3] = world[ current_pos[0]] [ current_pos[1] - 1];
		}
		int min;
		if (direction == -1){
			prev_direction = 1;
			direction = 1;
		}
		else	// give priority to the previous action
			action = 0;

		min = neighbors[direction];
		int i = 0;
		for (i = 0; i < 4; i++){
			if (neighbors[i] < min || (neighbors[i] <= min && i==1)){
				min = neighbors[i];
				direction = i;

				if ( direction - prev_direction == -1 || direction - prev_direction == -3)	// turn anticlock-wise
					action = 1;	// turn left
				else if ( direction - prev_direction == 1 || direction - prev_direction == 3) // turn right
					action = 2; // turn right
				else if (direction - prev_direction == 0) // continue straight
					action = 0;
				else	// turn around
					action = 3;
			}
		}
		action_array[action_array_size] = action;
		action_array_size++;
		// black previous cell
		world[current_pos[0]][current_pos[1]] = INT_MAX - 1;
		// update current position

		if (action == 0){	// continue straight
			if (prev_direction == 0)
				current_pos[0] = current_pos[0] - 1;
			else if (prev_direction == 1)
				current_pos[1] = current_pos[1] + 1;
			else if (direction == 2)
				current_pos[0] = current_pos[0] + 1;
			else if (prev_direction == 3)
				current_pos[1] = current_pos[1] - 1;
		}

		else if (action == 1){// turn left
				if (prev_direction == 0)
					current_pos[1] = current_pos[1] - 1;
				else if (prev_direction == 1)
					current_pos[0] = current_pos[0] - 1;
				else if (prev_direction == 2)
					current_pos[1] = current_pos[1] + 1;
				else if (prev_direction == 3)
					current_pos[0] = current_pos[0] + 1;
		}

		else if (action == 2){// turn right
			if (prev_direction == 0)
				current_pos[1] = current_pos[1] + 1;
			else if (prev_direction == 1)
				current_pos[0] = current_pos[0] + 1;
			else if (prev_direction == 2)
				current_pos[1] = current_pos[1] - 1;
			else if (prev_direction == 3)
				current_pos[0] = current_pos[0] - 1;
		}

		else if (action == 3){// turn around
			if (prev_direction == 0)
				current_pos[0] = current_pos[0] + 1;
			else if (prev_direction == 1)
				current_pos[1] = current_pos[1] - 1;
			else if (prev_direction == 2)
				current_pos[0] = current_pos[0] - 1;
			else if (prev_direction == 3)
				current_pos[1] = current_pos[1] + 1;
		}

		prev_direction = direction;
		print_world(current_pos[0], current_pos[1]);

		// check if goal reached
		int goalReached = 0;
		if (current_pos[1]== goals[0][0] && current_pos[0]== goals[0][1]){
			goalReached = 1;
			break;
		}
		if (goalReached)
			break;


	}
}

int get_min_manhattan_dist(int pos_x, int pos_y){
	int i, l, row, col;

	int dist_array[MAX_GOALS];
	for ( row = 0; row <= MAX_GOALS; row++){
		dist_array[row] = manhattan_distance(pos_y, goals[row][0], pos_x, goals[row][1]);
		if (pos_y == goals[0][0] && pos_x == goals[0][1])
				dist_array[row] = -1;
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
	int i, l, row, col;

    int min = dist_array[0];
	for ( i = 1; i < size; ++i)
	{
		if (dist_array[i] < min)
		{
			min = dist_array[i];
		}
	}
	return min;
}

void print_array(int array[], int length){
	int i, l, row, col;

    for ( i = 0; i < length; i++) {
 		printf("%d\t", array[i]);
    }
    printf("\n");
}

void print_2d_array(int array[][MAX_Y], int rows, int columns){
	int i, l, row, col;

    for ( i = 0; i < rows; i++) {
    	for ( l = 0; l < columns; l++) {
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

void print_world(int pos_x, int pos_y){
	int i, l, row, col;

    for ( i = 0; i <MAX_Y-1; i++) {
    	for ( l = 0; l < MAX_X-1; l++) {
    		if (world[i][l] == INT_MAX)
    			printf("INF\t");
    		else if (pos_x == i && pos_y == l)
    			printf("HERE\t");
    		else if (world[i][l] == INT_MAX-1)
    		    			printf("BLK\t");

    		else
    			printf("%d\t", world[i][l]);
    	}
    	printf("\n");
    }
	printf("\n");

}

int main(void) {
	int i, l, row, col;

	start_new[0] = start[0];
	start_new[1] = start[1];
	//world initialization
	for ( row=0; row < MAX_Y-1; row++){
			for ( col=0; col < MAX_X-1; col++){
				world[row][col] = -1;
			}

	}

	// translate coordinates to cell in the world
	start_pos[1] = round(start_new[0] / 0.305) - 1;
	start_pos[0] = (MAX_Y - 1) - round(start_new[1] / 0.305);

	// the four different goal positions
	goals[0][0] = round((goal[0] / 0.305))  - 1;
	goals[0][1] = round(MAX_Y - (goal[1] / 0.305))  - 1;
	goals[1][0] = round((goal[0] / 0.305))  - 1;
	goals[1][1]	= round(MAX_Y - (goal[1] / 0.305)) - 2;
	goals[2][0] = round((goal[0] / 0.305)) - 2;
	goals[2][1] = round(MAX_Y - (goal[1] / 0.305)) - 1;
	goals[3][0] = round((goal[0] / 0.305));
	goals[3][1] = round(MAX_Y - (goal[1] / 0.305)) - 1;
	goals[4][0] = round((goal[0] / 0.305)) - 1;
	goals[4][1] = round(MAX_Y - (goal[1] / 0.305));

	pathfinder();

	for (i = 0; i < action_array_size; i++){

	        if (action_array[i] == 0){
	        	ResetRotationCount(OUT_AC);
	            TermPrintln("Forward");
	            move_forward();
	            Wait(200);
	        }

	        else if (action_array[i] == 1){
	        	ResetRotationCount(OUT_AC);
				TermPrintln("Left");
				move_left();
				Wait(200);
			}

	        else if (action_array[i] == 2){
	        	ResetRotationCount(OUT_AC);
				TermPrintln("Right");
				move_right();
				Wait(200);
			}

	        else if (action_array[i] == 3){
				ResetRotationCount(OUT_AC);
				TermPrintln("Back");
				turn_back();
				Wait(200);
	        }

	    }

	return 0;
}


