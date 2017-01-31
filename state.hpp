#ifndef STATE_H
#define STATE_H
#include <stdlib.h>
#include <stdio.h>
#include <cstdlib>
#include <string>
#include <vector>
#include <stack>
#include "Map.hpp"

using namespace std;

// Position structure
struct Pos{
	int i;
	int j;
	Pos(int i, int j){
		this->i = i;
		this->j = j;
	}
};

// Manhattan distance between a and b
int manhattan(Pos a, Pos b);

// Compares two positions
bool compare_pos(Pos a, Pos b);

// Parsing string into position vector
void parse_pos(char *str,vector<Pos> *pos);

// State class definition
class State{

	public:

		// Pointer to parent
		State *parent;

		// Cost to reach from start
		int g;

		// Estimate path cost
		int f;

		// Action vector taken from parent
		string action_vector;

		// Robots positions
		vector<Pos> robots;

		// Boxes positions
		vector<Pos> boxes;

		// Constructor from parent
		State(State *parent,string action_vector);

		// Constructor from string descriptor
		State(char *robots_str, char *boxes_str);

		// Return stack with children states
		void expand(stack<State*> *children, Map *map);

		// Recursive action vector expansion
		void expand_action_vector(string action_vector, int i, char action, stack<State*> *children, Map *map);
		
		// Validate state against world rules
		bool validate(Map *map);

};

#endif
