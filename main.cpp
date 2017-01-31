#include <stdlib.h>
#include <stdio.h>
#include <list>
#include <cstdlib>
#include <iostream>
#include <string>
#include <ctime>
#include "state.hpp"

#define MAP_FILE	"map.csv"
#define NMAX		10000
#define EPSILON		1

// State display
void display(State *state, State *goal, Map *map);

// World display
void display_world(State *state, State *goal, Map *map);

// Clear list of states from memory
void clear_list(list<State*> *state_list,State* start, State* goal);

// Print plan
void print_plan(stack<State> plan, State* goal, Map *map);

// Compare two vectors
bool vector_equal(vector<Pos> veca, vector<Pos> vecb);

// Compare two states
bool state_equal(State *sta, State *stb, bool robot_pos);

// Heuristic
int heuristic(State *node, State *goal);

// Insert child to open list if correct conditions met
void new_child(State *child, list<State*> *open, list<State*> *closed, State *goal, int beamsize);

// Search for a plan
int search(State *start, State *goal, Map *map, stack<State> *plan, int beamsize);

int main(int argc, char **argv){

	Map *map = new Map(MAP_FILE);

	State *start	= new State("7,1,7,7","6,2,6,6");
	State *goal		= new State(NULL,"7,3,7,7");	

	printf("Start: ");
	display(start,NULL,map);

	printf("Goal: ");
	display(goal,NULL,map);

	// Running and taking execution time
	stack<State> plan;
	clock_t t_start = clock();
	int num_exp_nodes = search(start, goal, map, &plan, NMAX);
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
	}
	else
		printf("Plan failed.\n");

	return 0;

}

// State display
void display(State *state, State *goal, Map *map){
	printf("%s ",state->action_vector.c_str());
	for(Pos pos : state->robots)
		printf("(%d,%d)",pos.i,pos.j);
	printf(" : ");
	for(Pos pos : state->boxes)
		printf("(%d,%d)",pos.i,pos.j);
	printf("\n");
	display_world(state,goal,map);
	printf("\n");
}

// World display
void display_world(State *state, State *goal, Map *map){
	for(int i = map->rows-1; i >= 0; i--){
		printf("|");
		for(int j = 0; j < map->cols; j++){
			int element = (map->get_value(i,j))?'X':' ';
			if(goal != NULL)
				for(int k = 0; k < goal->boxes.size(); k++)
					if(compare_pos(Pos(i,j),goal->boxes.at(k))){
						element = k+97;
						break;
					}
			for(int k = 0; k < state->robots.size(); k++)
				if(compare_pos(Pos(i,j),state->robots.at(k))){
					element = k+48;
					break;
				}
			for(int k = 0; k < state->boxes.size(); k++)
				if(compare_pos(Pos(i,j),state->boxes.at(k))){
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
		printf("%3d: ",i++);
		display(&plan.top(),goal,map);
		plan.pop();
	}
}

// Check if two stacks are equal
bool vector_equal(vector<Pos> veca, vector<Pos> vecb){
	for(int i = 0; i < veca.size(); i++)
		if(!compare_pos(veca.at(i),vecb.at(i))) return false;
	return true;
}

// Compare two states
bool state_equal(State *sta, State *stb, bool robot_pos){
	if(robot_pos && !vector_equal(sta->robots, stb->robots)) return false;
	if(!vector_equal(sta->boxes, stb->boxes)) return false;
	return true;
}

// Heuristic
int heuristic(State *node, State *goal){

	// If number of boxes different, something is wrong
	if(node->boxes.size() != goal->boxes.size())
		return 0;

	// Sum of Manhattan distances
	int h = 0;
	for(int i = 0; i < node->boxes.size(); i++)
		h += manhattan(node->boxes.at(i),goal->boxes.at(i));

	return h;
}

// Insert child to open list if correct conditions met
void new_child(State *child, list<State*> *open, list<State*> *closed, State *goal, int beamsize){

	// Check if in the closed list
	for(State* node : *closed)
		if(state_equal(child,node,1)){
			delete child;
			return;
		}

	// Check if in the open list
	for(State* node : *open)
		if(state_equal(child,node,1)){
			if(child->g >= node->g){
				delete child;
				return;
			}else{
				open->remove(node);
				break;
			}
		}

	// Computing the heuristic
	int h = heuristic(child,goal);

	// Computing the estimated path cost
	child->f = child->g + EPSILON*h;

	// Inserting child into the sorted open list
	list<State*>::iterator it = open->begin();
	while(it != open->end())
		if((*(it++))->f > child->f) break;
	open->insert(it,child);

	// Beam search size limit to open list 
	if(open->size() > beamsize)
		open->pop_back();

}

// Search for a plan
int search(State *start, State *goal, Map *map, stack<State> *plan, int beamsize){

	stack<State*> children;
	list<State*> open;
	list<State*> closed;

	State *state = NULL;

	// Initializing open list
	open.push_back(start);
	
	// Search loop
	int num_exp_nodes = 0;
	for(;;){

		// Checking if failure
		if(num_exp_nodes++ == NMAX || open.empty()) return -1;

		// Visiting current node (least cost)
		state = open.front();
		open.pop_front();

		// Adding current node to the closed list
		closed.push_back(state);

		// Checking if goal found
		if(state_equal(state,goal,0)) break;

		// Expanding current node
		state->expand(&children,map);
		while(!children.empty()){
			new_child(children.top(),&open,&closed,goal,beamsize);
			children.pop();
		}
	}

	// Final path position
	plan->push(*state);

	// Defining path as a sequence of positions
	for(;;){

		// Check if path completed
		if(state_equal(state,start,1)) break;

		// Moving to parent node
		state = state->parent;
		
		// Inserting position in the path vector
		plan->push(*state);
	}

	// Clearing memory
	clear_list(&open,start,goal);
	clear_list(&closed,start,goal);

	return num_exp_nodes;

}
