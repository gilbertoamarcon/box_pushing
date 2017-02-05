#ifndef STATE_H
#define STATE_H
#include <stdlib.h>
#include <stdio.h>
#include <cstdlib>
#include <string>
#include <cmath>
#include <vector>
#include <stack>
#include "Map.hpp"
#include "Pos.hpp"

using namespace std;

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
		State(char *boxes_str, char *robots_str=NULL);

		// Compare with two states
		//  1: this > state
		//  0: this = state
		// -1: this < state
		static int compare(State *sta, State *stb);

		// Search for a state in a ordered state vector
		static bool binary_search(vector<State*> *vec, State *state);

		// Heuristic distance to goal
		int heuristic(State *goal);

		// Print state representation on console
		void print();

		// Return true io state equals goal
		bool is_goal(State *goal);

		// Return stack with children states
		void expand(stack<State*> *children, Map *map);

	private:

		// Recursive action vector expansion
		void expand_action_vector(string action_vector, int i, char action, stack<State*> *children, Map *map);
		
		// Validate state against world rules
		bool validate(Map *map);
};

#endif
