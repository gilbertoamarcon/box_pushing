#include "Search.hpp"

// int Search::num_exp_nodes;
// double Search::planning_time;

// int Search::max_iterations;
// int Search::beamsize;
// float Search::epsilon;

// stack<State> Search::plan;

Search::Search(State *startnode, State *goalnode){

	start = startnode;
	goal = goalnode;

}


// Loading problem from file
void Search::load_search_parameters(char *filename){

	// Search::test_var = 0;

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
	max_iterations = atoi(param_buffer);
	while(param_buffer[i] != ',') i++; i++;
	beamsize = atoi(param_buffer+i);
	while(param_buffer[i] != ',') i++; i++;
	epsilon = atof(param_buffer+i);

	// Done
	fclose(file);
	printf("Parameters loaded: max_iterations: %d, beamsize: %d, epsilon: %f\n",max_iterations, beamsize, epsilon);

	return;
}

// Print plan
void Search::print_plan(){
	int i = 0;
	printf("Plan: \n");
	vector<State> plan_cpy = plan;
	while(!plan_cpy.empty()){
		printf("%3d: %s",i++,plan_cpy.back().to_str());
		State::display_world(&plan_cpy.back());
		plan_cpy.pop_back();
	}
	printf("\n");
}

// Store plan execution history
void Search::store_plan(char *filename){

	// Checking if origin file exists
	FILE *file  = fopen(filename,"w");
	if(file == NULL){
		printf("Error: Origin file '%s' not found.\n",filename);
		return;
	}

	vector<State> plan_cpy = plan;
	while(!plan_cpy.empty()){
		fprintf(file,"%s",plan_cpy.back().to_str());
		plan_cpy.pop_back();
	}

	// Done
	fclose(file);
	printf("File '%s' stored.\n",filename);

	return;
}

// Insert child to open list if correct conditions met
void Search::new_child(State *child, list<State*> *open, vector<State*> *closed){

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
void Search::search(){

	clock_t t_start = clock();

	stack<State*> children;
	list<State*> open;
	vector<State*> closed;

	State *state = NULL;

	// Mark invalid positions for deadlock pruning
	State::map->set_Corners();
	State::map->set_Deadlocks(State::goal->boxes);

	// Initializing open list
	open.push_back(start);
	
	// Search loop
	num_exp_nodes = 0;
	for(;;){

		// Checking if failure
		if(num_exp_nodes++ == max_iterations || open.empty()){
			num_exp_nodes = -1;
			return;
		}

		// Visiting current node (least cost)
		state = open.front();
		open.pop_front();
		//printf("%d %d \n", state->robots[0].i, state->robots[0].j);

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
		state->expand(&children);
		while(!children.empty()){
			new_child(children.top(),&open,&closed);
			children.pop();
		}
	}


	// Final path position
	plan.push_back(*state);

	// Defining path as a sequence of positions

	for(;;){

		// Check if path completed
		if(State::compare(state,start) == 0) break;

		// Moving to parent node
		state = state->parent;
		
		// Inserting position in the path vector
		plan.push_back(*state);
	}

	// Clearing memory
	closed.clear();
	while(!open.empty()){
		delete open.front();
		open.pop_front();
	}

	
	planning_time = (double)(clock() - t_start)/(double)CLOCKS_PER_SEC;
}

void check_clashes(vector<vector<State>> plans){


}

