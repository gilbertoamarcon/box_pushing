#include "state.hpp"

// Manhattan distance between a and b
int manhattan(Pos a, Pos b){
	return abs(a.i - b.i) + abs(a.j - b.j);
}

// Compares two positions
bool compare_pos(Pos a, Pos b){
	return a.i == b.i && a.j == b.j;
}

// Parsing string into position vector
void parse_pos(char *str, vector<Pos> *pos){
	int i = 0;
	int j = 0;
	for(;;){
		i = atoi(str);
		while(str[0] != ',') str++; str++;
		j = atoi(str);
		pos->push_back(Pos(i,j));
		while(str[0] != ',' && str[0] != '\0') str++;
		if(str[0] == '\0') return;
		str++;
	}
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
	this->g += robots.size();
}

State::State(char *robots_str, char *boxes_str){

	this->parent = NULL;
	this->g = 0;
	this->f = 0;

	// Parsing robot positions
	if(robots_str != NULL)
		parse_pos(robots_str,&robots);

	// Parsing box positions
	if(boxes_str != NULL)
		parse_pos(boxes_str,&boxes);

	// Initializing action vector with no action
	for(Pos robot : this->robots)
		action_vector.push_back('N');

}

// Return stack with all valid children states
void State::expand(stack<State*> *children, Map *map){
	string action_vector;
	expand_action_vector(action_vector,robots.size(),NULL,children,map);
}

// Recursive action vector expansion
void State::expand_action_vector(string action_vector, int r, char action, stack<State*> *children, Map *map){

	// Adding action to action vector if not root node
	if(action != NULL)
		action_vector.push_back(action);

	// Leaf node, create and validate children 
	if(r-- == 0){
		State *child = new State(this,action_vector);
		if(child->validate(map))
			children->push(child);
		else
			delete child;
		return;
	}

	// Recursive expansion
	expand_action_vector(action_vector,r,'N',children,map);
	expand_action_vector(action_vector,r,'L',children,map);
	expand_action_vector(action_vector,r,'U',children,map);
	expand_action_vector(action_vector,r,'R',children,map);
	expand_action_vector(action_vector,r,'D',children,map);
}

// Validate state against world rules
bool State::validate(Map *map){

	vector<Pos> temp_boxes = boxes;

	// Displacing robots and moving boxes
	for(int i = 0; i < action_vector.size(); i++){

		// Displacing robot and moving boxes
		switch(action_vector.at(i)){

			// Moving left
			case 'L':

				// Robot displacement
				robots.at(i).j--;

				// Robot bounds checking
				if(robots.at(i).j < 0) return false;

				// Box displacement
				for(int j = 0; j < boxes.size(); j++)
					if(compare_pos(robots.at(i),boxes.at(j))){

						// Box displacing
						temp_boxes.at(j).j--;

						// Box-wall collision checking
						if(map->get_value(temp_boxes.at(j).i,temp_boxes.at(j).j)) return false;

						break;
					}

				break;

			// Moving up
			case 'U':

				// Robot displacement
				robots.at(i).i++;

				// Robot bounds checking
				if(robots.at(i).i > map->rows-1) return false;

				// Box displacement
				for(int j = 0; j < boxes.size(); j++)
					if(compare_pos(robots.at(i),boxes.at(j))){

						// Box displacing
						temp_boxes.at(j).i++;

						// Box-wall collision checking
						if(map->get_value(temp_boxes.at(j).i,temp_boxes.at(j).j)) return false;

						break;
					}

				break;

			// Moving right
			case 'R':

				// Robot displacement
				robots.at(i).j++;

				// Robot bounds checking
				if(robots.at(i).j > map->cols-1) return false;
				
				// Box displacement
				for(int j = 0; j < boxes.size(); j++)
					if(compare_pos(robots.at(i),boxes.at(j))){

						// Box displacing
						temp_boxes.at(j).j++;

						// Box-wall collision checking
						if(map->get_value(temp_boxes.at(j).i,temp_boxes.at(j).j)) return false;

						break;
					}

				break;

			// Moving down
			case 'D':

				// Robot displacement
				robots.at(i).i--;

				// Robot bounds checking
				if(robots.at(i).i < 0) return false;
				
				// Box displacement
				for(int j = 0; j < boxes.size(); j++)
					if(compare_pos(robots.at(i),boxes.at(j))){

						// Box displacing
						temp_boxes.at(j).i--;

						// Box-wall collision checking
						if(map->get_value(temp_boxes.at(j).i,temp_boxes.at(j).j)) return false;

						break;
					}

				break;
		}

		// Robot-wall collision checking
		if(map->get_value(robots.at(i).i,robots.at(i).j)) return false;
	}
	
	// Updating box states
	boxes = temp_boxes;

	// Checking collisions
	for(int i = 0; i < robots.size(); i++){
		
		// Checking robot-robot collisions
		for(int j = 0; j < i; j++)
			if(compare_pos(robots.at(i),robots.at(j))) return false;
		
		// Checking robot-box collisions
		for(Pos box : boxes)
			if(compare_pos(robots.at(i),box)) return false;

	}

	// State is valid
	return true;
}

