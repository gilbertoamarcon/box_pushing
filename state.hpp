#ifndef STATE_H
#define STATE_H
#include <stdlib.h>
#include <stdio.h>
#include <list>
#include <cstdlib>
#include <iostream>
#include <string>
#include <ctime>
#include <vector>
#include <stack>

using namespace std;

struct Pos{
	int i;
	int j;
	Pos(int i, int j){
		this->i = i;
		this->j = j;
	}
};

bool compare_pos(Pos a, Pos b);

// State class definition
class State{

	public:

		// Pointer to parent
		State *parent;

		// Cost to reach from start
		int g;

		// Estimate path cost
		int f;

		// Action taken from parent
		string action_vector;
		vector<Pos> robots;
		vector<Pos> boxes;

		// Constructor from parent
		State(State *parent,string action_vector);

		// Constructor from start/goal
		State(char *str);
		
		bool validate(int m, int n);

		void expand_action_vector(string action_vector, int i, char action, stack<State*> *children,int m, int n);

		// Return stack with children states
		void expand(stack<State*> *children,int m, int n);

};

#endif
