#include "Search.hpp"
#include <algorithm>
// int Search::num_exp_nodes;
// double Search::planning_time;

// int Search::max_iterations;
// float Search::epsilon;
// float Search::time_lim_secs;

// stack<State> Search::plan;


Search::Search(State *startnode, State *goalnode){
	start = startnode;
	goal = goalnode;
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

// Print plan static
void Search::print_plan(vector<State> p){
	int i = 0;
	printf("Plan: \n");
	vector<State> plan_cpy = p;
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

	// printf(divideAndConquer() ? "Yay!" : "Clashing Paths!");

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

		// Check if individual searches are possible
		// if(coupled)
		// 	if(checkReducedSolution(state))
		// 		break;
			// if (checkReducedSolution(state));
				// break;
		// Checking if goal found
		if(state->is_goal(goal, checkRobots)) break;

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

// Search for a plan
int Search::searchDecoupled(){

	clock_t t_start = clock();

	stack<State*> children;
	Closed closed;
	Open open;

	State *state = NULL;

	// Mark invalid positions for deadlock pruning
	State::map->set_Corners(State::goal->boxes);
	State::map->set_Deadlocks(State::goal->boxes);

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

		// Check if individual searches are possible
		if(checkReducedSolution(state))
			break;
		// Checking if goal found
		if(state->is_goal(goal, checkRobots)) break;

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

// Get a start & goal state, divide into subsearches and return results
bool Search::checkReducedSolution(State* cur){
	State *st1 = new State({cur->boxes[0]},{cur->robots[0]});
	State *go1 = new State({goal->boxes[0]},{goal->robots[0]});
	State *st2 = new State({cur->boxes[1]},{cur->robots[1]});
	State *go2 = new State({goal->boxes[1]},{goal->robots[1]});

	Search s1(st1, go1);
	s1.max_iterations = max_iterations;
	s1.epsilon = epsilon;
	s1.time_lim_secs = time_lim_secs;
	s1.search();
	// s1.print_plan();

	Search s2(st2, go2);
	s2.max_iterations = max_iterations;
	s2.epsilon = epsilon;
	s2.time_lim_secs = time_lim_secs;
	s2.search();
	// s2.print_plan();

	bool result = Search::paths_free({s1.plan,s2.plan});
	// if (result){
	// 	printf("INDV SOLUTIONS: ======================");
	// 	s1.print_plan();
	// 	s2.print_plan();
	// }	
	// printf(result ? "FREE!-----------\n":"...\n");
	return result;
}

bool Search::paths_free(vector<vector<State>> plans){
	for(int i=0; i<plans.size(); i++){
		for(int j=0; j<plans.size(); j++){
			if (i<j){
				vector<State> plan1 = plans[i];
				vector<State> plan2 = plans[j];
				// for (int k=0; k<std::min(plan1.size(), plan2.size()); k++){
					// printf("Plan1: %s", plan1[k].to_str().c_str());
					// printf("Plan2: %s", plan2[k].to_str().c_str());
					// if (State::is_Clashing(&plan1[k], &plan2[k]))
						// printf("Clashing! \n");
				// }
				for (int k=0; k<std::min(plan1.size(), plan2.size()); k++){
					// printf(plan1[k].to_str().c_str());
					// printf(plan2[k].to_str().c_str());
					if (State::is_Clashing(&plan1[k], &plan2[k]))
						return false;
				}
			}
		}
	}
	return true;
}

vector<State*> Search::returnClashes(vector<vector<State>> plans){
	// Takes plans as parameters, if the plans clash; returns a vector of
	// the state just before the first clash, and the state just after the next one
	// If no clash, return empty vector
	vector<int> clashes = {};
	for(int i=0; i<plans.size(); i++){
		for(int j=0; j<plans.size(); j++){
			if (i<j){
				vector<State> plan1 = plans[i];
				vector<State> plan2 = plans[j];
				for (int k=0; k<std::min(plan1.size(), plan2.size()); k++){
					printf("Plan1: %s", plan1[k].to_str().c_str());
					printf("Plan2: %s", plan2[k].to_str().c_str());
					if (State::is_Clashing(&plan1[k], &plan2[k])){
						printf("Clashing! \n");
						clashes.push_back(k);}
				}
			}
		}
	}
	if (clashes.size()==0)
		return {};

	// for (int i=0;i<clashes.size();i++)
	//  	printf("%d \n", clashes[i]);

	int max = *std::max_element(clashes.begin(), clashes.end());
	int min = *std::min_element(clashes.begin(), clashes.end());


	// THIS PART OF THE CODE ASSUMES 2 AGENTS 2 BOXES
	vector<State> plan1 = plans[0];
	vector<State> plan2 = plans[1];	

	State *st1 = &plan1[max+1];
	State *go1 = &plan1[min-1];
	State *st2 = &plan2[max+1];
	State *go2 = &plan2[min-1];

	printf("Clashing Starts: %s", plan1[max+1].to_str().c_str());
	printf("Clashing Starts: %s", plan2[max+1].to_str().c_str());

	printf("Clashing Ends: %s", plan1[min-1].to_str().c_str());
	printf("Clashing Ends: %s", plan2[min-1].to_str().c_str());

	State *stc = new State(st1,"");
	*stc = *st1;
	stc->boxes.insert(stc->boxes.end(), st2->boxes.begin(), st2->boxes.end());
	stc->robots.insert(stc->robots.end(), st2->robots.begin(), st2->robots.end());
	State *goc = new State(st1,"");
	*goc = *go1;
	goc->boxes.insert(goc->boxes.end(), go2->boxes.begin(), go2->boxes.end()); 
	goc->robots.insert(goc->robots.end(), go2->robots.begin(), go2->robots.end());

	printf("START COUPLED SEARCH: %s", stc->to_str().c_str());
	printf("END COUPLED SEARCH: %s", goc->to_str().c_str());

	printf("Address: %d \n", stc);
	printf("Address: %d \n", goc);

	return {stc, goc};

}




clashInfo Search::returnClashesStruct(vector<vector<State>> plans){
	// Takes plans as parameters, if the plans clash; returns a vector of
	// the state just before the first clash, and the state just after the next one
	// If no clash, return empty vector
	clashInfo clash;
	vector<int> clashes = {};

	for(int i=0; i<plans.size(); i++){
		for(int j=0; j<plans.size(); j++){
			if (i<j){
				vector<State> plan1 = plans[i];
				vector<State> plan2 = plans[j];
				for (int k=0; k<std::min(plan1.size(), plan2.size()); k++){
					if (State::is_Clashing(&plan1[k], &plan2[k])){
						clashes.push_back(k);}
				}
			}
		}
	}
	if (clashes.size()==0){
		clash.isClashing=false;
		return clash;
	}


	int max = *std::max_element(clashes.begin(), clashes.end());
	int min = *std::min_element(clashes.begin(), clashes.end());


	// THIS PART OF THE CODE ASSUMES 2 AGENTS 2 BOXES
	vector<State> plan1 = plans[0];
	vector<State> plan2 = plans[1];	

	State *st1 = &plan1[max+1];
	State *go1 = &plan1[min-1];
	State *st2 = &plan2[max+1];
	State *go2 = &plan2[min-1];

	printf("Clashing Starts: %s", plan1[max+1].to_str().c_str());
	printf("Clashing Starts: %s", plan2[max+1].to_str().c_str());

	printf("Clashing Ends: %s", plan1[min-1].to_str().c_str());
	printf("Clashing Ends: %s", plan2[min-1].to_str().c_str());

	clash.t_start = max+1;
	clash.t_end = min-1;

	// State *stc = new State(st1,"");
	// State *stc = new State();
	// *stc = *st1;
	// stc->boxes.insert(stc->boxes.end(), st2->boxes.begin(), st2->boxes.end());
	// stc->robots.insert(stc->robots.end(), st2->robots.begin(), st2->robots.end());
	// State *goc = new State();
	// *goc = *go1;
	// goc->boxes.insert(goc->boxes.end(), go2->boxes.begin(), go2->boxes.end()); 
	// goc->robots.insert(goc->robots.end(), go2->robots.begin(), go2->robots.end());

	// printf("START COUPLED SEARCH: %s", stc->to_str().c_str());
	// printf("END COUPLED SEARCH: %s", goc->to_str().c_str());

	// State* s = State::combine_states({st1,st2});
	// printf()->to_str().c_str());
	// printf(State::combine_states({go1,go2})->to_str().c_str());

	// clash.clashes = {stc, goc};
	clash.clashes = {State::combine_states({st1,st2}), State::combine_states({go1,go2})};
	return clash;

}