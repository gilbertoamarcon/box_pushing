#include "State.hpp"

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

State::State(char *boxes_str,char *robots_str){

	this->parent = NULL;
	this->g = 0;
	this->f = 0;

	// Parsing box positions
	if(boxes_str != NULL)
		Pos::parse(boxes_str,&boxes);

	// Parsing robot positions
	if(robots_str != NULL)
		Pos::parse(robots_str,&robots);

	// Initializing action vector with no action
	for(Pos robot : this->robots)
		action_vector.push_back('N');

}

// Compare with two states
//  1: this > state
//  0: this = state
// -1: this < state
int State::compare(State *sta, State *stb){
	int aux = 0;
	aux = Pos::compare_vec(sta->boxes,stb->boxes);
	if(aux != 0) return aux;
	aux = Pos::compare_vec(sta->robots,stb->robots);
	return aux;
}

// Binary state search
bool State::binary_search(vector<State*> *vec, State *state){
	int aux = 0;
	int m = 0;
	int l = 0;
	int r = vec->size()-1;
	for(;;){
		if(l > r)
			return false;
		m = floor((l+r)/2);
		aux = State::compare(vec->at(m),state);
		if(aux == 0)
			return true;
		if(aux ==  1)
			r = m-1;
		else
			l = m+1;
	}
}

// Heuristic
int State::heuristic(State *goal){

	// If number of boxes different, something is wrong
	if(boxes.size() != goal->boxes.size())
		return 0;

	int max_dist = 0;	// Max Manhattan distance 
	int sum_dist = 0;	// Sum of all Manhattan distances
	for(int i = 0; i < boxes.size(); i++){
		int dist = Pos::manhattan(boxes.at(i),goal->boxes.at(i));
		sum_dist += dist;
		if(dist > max_dist)
			max_dist = dist;
	}

	// Manhattan distance per robot
	int dist_per_robot = ceil(((double)sum_dist)/robots.size());

	if(dist_per_robot > max_dist)
		return dist_per_robot;
	return max_dist;
}

// Print state representation on console
char* State::sprint(){
	char buffer[BUFFER_SIZE];
	sprintf(buffer,"%s ",action_vector.c_str());
	for(Pos pos : boxes) sprintf(buffer,"%s%s ",buffer,pos.sprint());
	sprintf(buffer,":");
	for(Pos pos : robots) sprintf(buffer,"%s%s ",buffer,pos.sprint());
	sprintf(buffer,"\n");
	return buffer;
}

// Check if two states are equal
bool State::is_goal(State *goal){
	return Pos::compare_vec(boxes,goal->boxes) == 0;
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
					if(Pos::compare(robots.at(i),boxes.at(j))){

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
					if(Pos::compare(robots.at(i),boxes.at(j))){

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
					if(Pos::compare(robots.at(i),boxes.at(j))){

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
					if(Pos::compare(robots.at(i),boxes.at(j))){

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
			if(Pos::compare(robots.at(i),robots.at(j))) return false;
		
		// Checking robot-box collisions
		for(Pos box : boxes)
			if(Pos::compare(robots.at(i),box)) return false;

	}

	// State is valid
	return true;
}

