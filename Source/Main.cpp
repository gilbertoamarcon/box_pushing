#include <stdlib.h>
#include <stdio.h>
#include <list>
#include <cstdlib>
#include <iostream>
#include <string>
#include <ctime>
#include <vector>
#include "State.hpp"

// File names
#define PROB_FILE	"Files/problem.csv"
#define MAP_FILE	"Files/map.csv"
#define PLAN_FILE	"Files/plan.csv"
#define CFG_FILE	"Files/cfg.csv"

// World print
void display_world(State *state, State *goal, Map *map);

// Clear list of states from memory
void clear_list(list<State*> *state_list,State* start, State* goal);

// Print plan
void print_plan(stack<State> plan, State* goal, Map *map);

// Store plan execution history
void store_plan(char *filename, stack<State> plan);

// Loading configuration from file
void load_cfg(char *filename, int &max_exp, int &beamsize, float &epsilon);

// Loading problem from file
void load_problem(char *filename, State **start, State **goal);

// Insert child to open list if correct conditions met
void new_child(State *child, list<State*> *open, vector<State*> *closed, State *goal, int beamsize, float epsilon);

// Search for a plan
int search(State *start, State *goal, Map *map, stack<State> &plan, int max_exp, int beamsize, float epsilon);

int main(int argc, char **argv){


	// Search configuration parameters
	int max_exp		= 0;
	int beamsize	= 0;
	float epsilon	= 0;
	load_cfg(CFG_FILE, max_exp, beamsize, epsilon);

	// Problem parameters
	State *start	= NULL;
	State *goal		= NULL;
	load_problem(PROB_FILE, &start, &goal);

	Map *map = new Map(MAP_FILE);

	// Running and taking execution time
	stack<State> plan;
	clock_t t_start = clock();
	int num_exp_nodes = search(start, goal, map, plan, max_exp, beamsize, epsilon);
	double planning_time = (double)(clock() - t_start)/(double)CLOCKS_PER_SEC;

	// Presenting results on screen
	if(num_exp_nodes > 0){
		printf("Plan found.\n");
		printf("%d expanded nodes.\n",num_exp_nodes);
		printf("%d actions.\n",plan.size());
		printf("%f seconds.\n",planning_time);
		printf("Plan: \n");
		print_plan(plan,goal,map);
		printf("\n");
		store_plan(PLAN_FILE,plan);
	}
	else
		printf("Plan failed.\n");

	return 0;

}

// Loading problem from file
void load_cfg(char *filename, int &max_exp, int &beamsize, float &epsilon){

	// Checking if origin file exists
	FILE *file  = fopen(filename,"r");
	if(file == NULL){
		printf("Error: Origin file '%s' not found.\n",filename);
		return;
	}

	// Getting paramters
	char param_buffer[BUFFER_SIZE];
	fgets(param_buffer,BUFFER_SIZE,file);
	int i = 0;
	max_exp = atoi(param_buffer);
	while(param_buffer[i] != ',') i++; i++;
	beamsize = atoi(param_buffer+i);
	while(param_buffer[i] != ',') i++; i++;
	epsilon = atof(param_buffer+i);

	// Done
	fclose(file);
	printf("Parameters loaded: max_exp: %d, beamsize: %d, epsilon: %f\n",max_exp, beamsize, epsilon);

	return;
}

// Loading problem from file
void load_problem(char *filename, State **start, State **goal){

	// Checking if origin file exists
	FILE *file  = fopen(filename,"r");
	if(file == NULL){
		printf("Error: Origin file '%s' not found.\n",filename);
		return;
	}
	
	// Getting problem conditions
	char init_robot[BUFFER_SIZE];
	char init_boxes[BUFFER_SIZE];
	char final_boxes[BUFFER_SIZE];
	fgets(init_robot,BUFFER_SIZE,file);
	fgets(init_boxes,BUFFER_SIZE,file);
	fgets(final_boxes,BUFFER_SIZE,file);

	// Initializing start and goal states
	*start	= new State(init_boxes,init_robot);
	*goal	= new State(final_boxes);

	// Done
	fclose(file);
	printf("Start: %s",(*start)->sprint());
	printf("Goal: %s",(*goal)->sprint());

	return;
}

// World print
void display_world(State *state, State *goal, Map *map){
	for(int i = map->rows-1; i >= 0; i--){
		printf("|");
		for(int j = 0; j < map->cols; j++){
			int element = (map->get_value(i,j))?'X':' ';
			if(goal != NULL)
				for(int k = 0; k < goal->boxes.size(); k++)
					if(Pos::compare(Pos(i,j),goal->boxes.at(k))){
						element = k+97;
						break;
					}
			for(int k = 0; k < state->robots.size(); k++)
				if(Pos::compare(Pos(i,j),state->robots.at(k))){
					element = k+48;
					break;
				}
			for(int k = 0; k < state->boxes.size(); k++)
				if(Pos::compare(Pos(i,j),state->boxes.at(k))){
					element = k+65;
					break;
				}
			printf(" %c ",element);
		}
		printf("|\n");
	}
	printf("\n");
}

// Clear list
void clear_list(list<State*> *state_list,State* start, State* goal){
	while(!state_list->empty()){
		if(state_list->front() != start && state_list->front() != goal)
			delete state_list->front();
		state_list->pop_front();
	}
}

// Print plan
void print_plan(stack<State> plan, State* goal, Map *map){
	int i = 0;
	while(!plan.empty()){
		printf("%3d: %s",i++,plan.top().sprint());
		display_world(&plan.top(),goal,map);
		plan.pop();
	}
}

// Store plan execution history
void store_plan(char *filename, stack<State> plan){

	// Checking if origin file exists
	FILE *file  = fopen(filename,"w");
	if(file == NULL){
		printf("Error: Origin file '%s' not found.\n",filename);
		return;
	}

	while(!plan.empty()){
		fprintf(file,"%s",plan.top().sprint());
		plan.pop();
	}

	// Done
	fclose(file);
	printf("File '%s' stored.\n",filename);

	return;
}

// Insert child to open list if correct conditions met
void new_child(State *child, list<State*> *open, vector<State*> *closed, State *goal, int beamsize, float epsilon){

	// Checking if in the closed list
	if(State::binary_search(closed,child)){
		delete child;
		return;
	}

	// Check if in the open list
	for(State* node : *open)
		if(State::compare(child,node) == 0){
			if(child->g >= node->g){
				delete child;
				return;
			}else{
				open->remove(node);
				break;
			}
		}

	// Computing the heuristic
	int h = child->heuristic(goal);

	// Computing the estimated path cost
	child->f = child->g + epsilon*h;

	// Inserting child into the sorted open list
	list<State*>::iterator it = open->begin();
	for(it = open->begin(); it != open->end(); it++)
		if((*it)->f >= child->f) break;
	open->insert(it,child);

	// Beam search size limit to open list 
	if(open->size() > beamsize)
		open->pop_back();

}

// Search for a plan
int search(State *start, State *goal, Map *map, stack<State> &plan, int max_exp, int beamsize, float epsilon){

	stack<State*> children;
	list<State*> open;
	vector<State*> closed;

	State *state = NULL;

	// Initializing open list
	open.push_back(start);
	
	// Search loop
	int num_exp_nodes = 0;
	for(;;){

		// Checking if failure
		if(num_exp_nodes++ == max_exp || open.empty()) return -1;

		// Visiting current node (least cost)
		state = open.front();
		open.pop_front();

		// Inserting current node into the sorted closed list
		vector<State*>::iterator it = closed.begin();
		for(it = closed.begin(); it != closed.end(); it++){
			int aux = State::compare((*it),state);
			if(aux ==  0 || aux == 1) break;
		}
		closed.insert(it,state);

		// Checking if goal found
		if(state->is_goal(goal)) break;

		// Expanding current node
		state->expand(&children,map);
		while(!children.empty()){
			new_child(children.top(),&open,&closed,goal,beamsize,epsilon);
			children.pop();
		}
	}

	// Final path position
	plan.push(*state);

	// Defining path as a sequence of positions
	for(;;){

		// Check if path completed
		if(State::compare(state,start) == 0) break;

		// Moving to parent node
		state = state->parent;
		
		// Inserting position in the path vector
		plan.push(*state);
	}

	// Clearing memory
	clear_list(&open,start,goal);

	return num_exp_nodes;

}
