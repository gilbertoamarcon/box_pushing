#include "Search.hpp"

// int Search::num_exp_nodes;
// double Search::planning_time;

// int Search::max_iterations;
// float Search::epsilon;
// float Search::time_lim_secs;

// stack<State> Search::plan;


Search::Search(State *startnode, State *goalnode){

	printf("A: %d ", start);
	printf("B: %d ", startnode);
	start = startnode;
	goal = goalnode;
	printf("A: %d ", start);
	printf("B: %d ", startnode);
	printf(start->to_str().c_str());
	printf(goal->to_str().c_str());

}

// Loading problem from file
void Search::load_search_parameters(char *filename){

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
	epsilon = atof(param_buffer+i);
	while(param_buffer[i] != ',') i++; i++;
	time_lim_secs = atof(param_buffer+i);

	// Done
	fclose(file);

	return;
}

// Print plan
void Search::print_plan(){
	int i = 0;
	printf("Plan: \n");
	vector<State> plan_cpy = plan;
	while(!plan_cpy.empty()){
		printf("%3d: %s",i++,plan_cpy.back().to_str().c_str());
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
		fprintf(file,"%s",plan_cpy.back().to_str().c_str());
		plan_cpy.pop_back();
	}

	// Done
	fclose(file);

	return;
}

// Insert child to open vector if correct conditions met
void Search::new_child(State *child, Open *open, Closed *closed){

	// Checking if in the closed vector
	if(closed->find(child)){
		delete child;
		return;
	}

	// Computing the heuristic
	int h = child->heuristic(State::goal);

	// Computing the estimated path cost
	child->f = child->g + epsilon*h;

	open->insert(child);

}

// Search for a plan
int Search::search(){

	clock_t t_start = clock();

	stack<State*> children;
	Closed closed;
	Open open;

	State *state = NULL;

	// Mark invalid positions for deadlock pruning
	State::map->set_Corners(State::goal->boxes);
	State::map->set_Deadlocks(State::goal->boxes);
	// printf(State::goal->to_str().c_str());
	// printf(State::start->to_str().c_str());
	// printf(start->to_str().c_str());
	// printf(goal->to_str().c_str());

	// Initializing open vector
	open.insert(start);

	// Search loop
	num_exp_nodes = 0;
	for(;;){

		// Failure: impossible problem
		if(open.empty()){
			num_exp_nodes = -1;
			return 1;
		}

		// Failure: time out
		if((double)(clock() - t_start)/(double)CLOCKS_PER_SEC > time_lim_secs){
			num_exp_nodes = -1;
			return 2;
		}

		// Failure: exceeded node expansion limit
		if(num_exp_nodes++ == max_iterations){
			num_exp_nodes = -1;
			return 3;
		}

		// Visiting current node (least cost)
		state = open.pop();

		// Inserting current node into the sorted closed vector
		closed.insert(state);

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
	
	planning_time = (double)(clock() - t_start)/(double)CLOCKS_PER_SEC;

	return 0;

}

bool Search::paths_free(vector<vector<State>> plans){
	for(int i=0; i<plans.size(); i++){
		for(int j=0; j<plans.size(); j++){
			if (i<j){
				vector<State> plan1 = plans[i];
				vector<State> plan2 = plans[j];
				for (int k=0; k<std::min(plan1.size(), plan2.size()); k++){
					printf(plan1[k].to_str().c_str());
					printf(plan2[k].to_str().c_str());
					if (State::is_Clashing(&plan1[k], &plan2[k]))
						return false;
				}
			}
		}
	}
	return true;
}