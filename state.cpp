#include "state.hpp"


bool compare_pos(Pos a, Pos b){
	return a.i == b.i && a.j == b.j;
}

// Constructor from parent
State::State(State *parent,string action_vector){

	// Copy from parent
	*this = *parent;

	// Pointer to parent
	this->parent = parent;

	// Action from parent
	this->action_vector = action_vector;

	// Cost to reach from goal
	this->g++;
}

// Constructor from start/goal
State::State(char *str){
	this->parent = NULL;
	this->g = 0;
	this->f = 0;

	// Parsing state description string
	int i = 0;
	int j = 0;
	for(;;){
		i = atoi(str);
		while(str[0] != ',') str++; str++;
		j = atoi(str);
		robots.push_back(Pos(i,j));
		while(str[0] != ',' && str[0] != ';') str++;
		if(str[0] == ';') break;
		str++;
	}
	str++;
	for(;;){
		i = atoi(str);
		while(str[0] != ',') str++; str++;
		j = atoi(str);
		boxes.push_back(Pos(i,j));
		while(str[0] != ',' && str[0] != '\0') str++;
		if(str[0] == '\0') break;
		str++;
	}

	for(Pos robot : robots)
		action_vector.push_back('N');

}

bool State::validate(int m, int n){
	vector<Pos> temp_boxes = boxes;
	for(int i = 0; i < action_vector.size(); i++){
		switch(action_vector.at(i)){
			case 'L':
				robots.at(i).j--;
				if(robots.at(i).j < 0) return false;
				for(int j = 0; j < boxes.size(); j++)
					if(compare_pos(robots.at(i),boxes.at(j)))
						temp_boxes.at(j).j--;
				break;
			case 'U':
				robots.at(i).i++;
				if(robots.at(i).i > m-1) return false;
				for(int j = 0; j < boxes.size(); j++)
					if(compare_pos(robots.at(i),boxes.at(j)))
						temp_boxes.at(j).i++;
				break;
			case 'R':
				robots.at(i).j++;
				if(robots.at(i).j > n-1) return false;
				for(int j = 0; j < boxes.size(); j++)
					if(compare_pos(robots.at(i),boxes.at(j)))
						temp_boxes.at(j).j++;
				break;
			case 'D':
				robots.at(i).i--;
				if(robots.at(i).i < 0) return false;
				for(int j = 0; j < boxes.size(); j++)
					if(compare_pos(robots.at(i),boxes.at(j)))
						temp_boxes.at(j).i--;
				break;
		}
	}
	boxes = temp_boxes;
	for(int i = 0; i < robots.size(); i++){
		for(int j = 0; j < i; j++)
			if(compare_pos(robots.at(i),robots.at(j))) return false;
		for(Pos box : boxes)
			if(compare_pos(robots.at(i),box)) return false;
	}
	return true;
}

void State::expand_action_vector(string action_vector, int r, char action, stack<State*> *children,int m, int n){
	action_vector.push_back(action);
	if(--r == 0) {
		State *child = new State(this,action_vector);
		if(child->validate(m,n))
			children->push(child);
		else
			delete child;
		return;
	}
	expand_action_vector(action_vector,r,'N',children,m,n);
	expand_action_vector(action_vector,r,'L',children,m,n);
	expand_action_vector(action_vector,r,'U',children,m,n);
	expand_action_vector(action_vector,r,'R',children,m,n);
	expand_action_vector(action_vector,r,'D',children,m,n);
}


// Return stack with children states
void State::expand(stack<State*> *children,int m, int n){
	int r = robots.size();
	string action_vector;
	expand_action_vector(action_vector,r,'N',children,m,n);
	expand_action_vector(action_vector,r,'L',children,m,n);
	expand_action_vector(action_vector,r,'U',children,m,n);
	expand_action_vector(action_vector,r,'R',children,m,n);
	expand_action_vector(action_vector,r,'D',children,m,n);
}

