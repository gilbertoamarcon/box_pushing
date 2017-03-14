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
		if(coupled)
			if(checkReducedSolution(state))
				break;
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
int Search::searchDecoupled1(){
	//SPLICE SEARCH
	printf("Distributing Tasks...\n");
	State *st1 = new State({start->boxes[0]},{start->robots[0]});
	State *st2 = new State({start->boxes[1]},{start->robots[1]});
	State *go1 = new State();
	State *go2 = new State();
	go1->boxes.push_back(goal->boxes[0]);
	go2->boxes.push_back(goal->boxes[1]);
	if (checkRobots){
		go1->robots.push_back(goal->robots[0]);
		go2->robots.push_back(goal->robots[1]);
	}

	//SEARCH INDIVIDUALLY
	printf("Searching Individually...\n");

	Search *s1 = new Search(st1, go1);
	copySearchParameters(s1);
	s1->search();

	Search *s2 = new Search(st2, go2);
	copySearchParameters(s2);
	s2->search();

	//CHECK COLLISION SET
	clashInfo c;
	Search *sc = new Search(State::start, State::goal);
	for(int i=1;;i++){
	printf("Checking Collision Set... BT: %d \n", i);
	c = Search::returnClashesStruct({s1->plan,s2->plan}, i);

	// printf("CLASHING START_END: %d %d \n",c.t_start,c.t_end);
	//DO COUPLED SOLUTION IF CLASHING

	if (c.clashes.size() == 0)
		printf("No clashes between individual plans, executing paths...\n");
	else
		printf("Doing local coupled search...\n");
		sc->start = c.clashes[0];
		sc->goal = c.clashes[1];
		printf(c.clashes[0]->to_str().c_str());
		printf(c.clashes[1]->to_str().c_str());
		State::display_world(c.clashes[0]);
		State::display_world(c.clashes[1]);
		// Search sc(c.clashes[0], god);
		copySearchParameters(sc);
		sc->checkRobots = true;
		// sc->coupled = true;
		if(sc->search()==0){
			sc->print_plan();
			break;
		}
	}

	plan = Search::mergePlans(s1, s2, sc, c);
}

vector<State> Search::mergePlans(Search* s1, Search* s2, Search* sc, clashInfo c){
	//ASSUMES 2 PATHS
	//TIE ALL THE RESULTING PATHS
	// printf("Displaying Final Path...\n");
	// printf("%d %d %d %d",s1->plan.begin(),s1->plan.begin(),c.t_start,c.t_end);
	vector<State> p1(s1->plan.begin() + c.t_start + 1, s1->plan.end()); 
	std::reverse(p1.begin(), p1.end());
	vector<State> p2(s2->plan.begin() + c.t_start + 1, s2->plan.end()); 	
	std::reverse(p2.begin(), p2.end());
	vector<State> p3(s1->plan.begin(), s1->plan.begin() + c.t_end);
	std::reverse(p3.begin(), p3.end()); 
	vector<State> p4(s2->plan.begin(), s2->plan.begin() + c.t_end); 
	std::reverse(p4.begin(), p4.end());
	vector<State> p5(sc->plan);
	std::reverse(p5.begin(), p5.end());

	printf("IND path1 ...\n");
	for (int i=0;i<p1.size();i++)
		printf("%s \n",p1[i].to_str().c_str());
	printf("IND path2 ...\n");
	for (int i=0;i<p2.size();i++)
		printf("%s \n",p2[i].to_str().c_str());
	printf("IND path1_end...\n");
	for (int i=0;i<p3.size();i++)
		printf("%s \n",p3[i].to_str().c_str());
	printf("IND path2_end...\n");
	for (int i=0;i<p4.size();i++)
		printf("%s \n",p4[i].to_str().c_str());
	printf("Coupled mid path...\n");
	for (int i=0;i<p5.size();i++)
		printf("%s \n",p5[i].to_str().c_str());

	// printf("Merged INDV PATHS1...\n");
	//MERGE INDV PATHS
	int max1 = std::max(p1.size(), p2.size());
	vector<State> pp1 = {};
	for (int i=0;i<max1;i++)
		pp1.push_back(*State::combine_states({&(p1[i]),&(p2[i])}));

	// for (int i=0;i<pp1.size();i++)
	// 	printf("%s \n",pp1[i].to_str().c_str());

	// printf("Merged INDV PATHS2...\n");
	//MERGE INDV PATHS
	int max2 = std::max(p3.size(), p4.size());
	vector<State> pp2 = {};
	for (int i=0;i<max2;i++)
		pp2.push_back(*State::combine_states({&(p3[i]),&(p4[i])}));

	// for (int i=0;i<pp2.size();i++)
	// 	printf("%s \n",pp2[i].to_str().c_str());

	//ACTUAL MERGING
	pp1.insert(pp1.end(), p5.begin(), p5.end()); 
	pp1.insert(pp1.end(), pp2.begin(), pp2.end()); 

	// // printf("FINAL PATH...\n");
	// // for (int i=0;i<pp1.size();i++)
	// // 	printf("%s \n",pp1[i].to_str().c_str());

	std::reverse(pp1.begin(), pp1.end());
	
	return pp1;
}

// vector<State> mergePlans(Search s1, Search s2, Search sc){
// 	//ASSUMES 2 PATHS
// 	//TIE ALL THE RESULTING PATHS
// 	printf("Displaying Final Path...\n");
// 	printf("%d %d %d %d",s1->plan.begin(),s1->plan.begin(),c.t_start,c.t_end);
// 	vector<State> p1(s1->plan.begin() + c.t_start, s1->plan.end()); 
// 	std::reverse(p1.begin(), p1.end());
// 	vector<State> p2(s2->plan.begin() + c.t_start, s2->plan.end()); 	
// 	std::reverse(p2.begin(), p2.end());
// 	vector<State> p3(s1->plan.end() - c.t_end, s1->plan.end());
// 	std::reverse(p3.begin(), p3.end()); 
// 	vector<State> p4(s2->plan.end() - c.t_end, s2->plan.end()); 
// 	std::reverse(p4.begin(), p4.end());
// 	vector<State> p5(sc->plan);
// 	std::reverse(p5.begin(), p5.end());

// 	printf("IND path1 ...\n");
// 	for (int i=0;i<p1.size();i++)
// 		printf("%s \n",p1[i].to_str().c_str());
// 	printf("IND path2 ...\n");
// 	for (int i=0;i<p2.size();i++)
// 		printf("%s \n",p2[i].to_str().c_str());
// 	printf("IND path1_end...\n");
// 	for (int i=0;i<p3.size();i++)
// 		printf("%s \n",p3[i].to_str().c_str());
// 	printf("IND path2_end...\n");
// 	for (int i=0;i<p4.size();i++)
// 		printf("%s \n",p4[i].to_str().c_str());
// 	printf("Coupled mid path...\n");
// 	for (int i=0;i<p5.size();i++)
// 		printf("%s \n",p5[i].to_str().c_str());

// 	printf("Merged INDV PATHS1...\n");
// 	//MERGE INDV PATHS
// 	int max1 = std::max(p1.size(), p2.size());
// 	vector<State> pp1 = {};
// 	for (int i=0;i<max1;i++)
// 		pp1.push_back(*State::combine_states({&(p1[i]),&(p2[i])}));

// 	for (int i=0;i<pp1.size();i++)
// 		printf("%s \n",pp1[i].to_str().c_str());

// 	printf("Merged INDV PATHS2...\n");
// 	//MERGE INDV PATHS
// 	int max2 = std::max(p3.size(), p4.size());
// 	vector<State> pp2 = {};
// 	for (int i=0;i<max2;i++)
// 		pp1.push_back(*State::combine_states({&(p3[i]),&(p4[i])}));

// 	for (int i=0;i<pp2.size();i++)
// 		printf("%s \n",pp2[i].to_str().c_str());

// 	//ACTUAL MERGING
// 	//pp1+p5+pp2
// 	pp1.insert(pp1.end(), p5.begin(), p5.end()); 
// 	pp1.insert(pp1.end(), pp2.begin(), pp2.end()); 
// 	printf("FINAL PATH...\n");
// 	for (int i=0;i<pp1.size();i++)
// 		printf("%s \n",pp1[i].to_str().c_str());

// 	std::reverse(pp1.begin(), pp1.end());
// 	Search::print_plan(pp1);
// }

// Get a start & goal state, divide into subsearches and return results
bool Search::checkReducedSolution(State* cur){
	State *st1 = new State({cur->boxes[0]},{cur->robots[0]});
	State *st2 = new State({cur->boxes[1]},{cur->robots[1]});
	State *go1 = new State();
	State *go2 = new State();

	go1->boxes.push_back(goal->boxes[0]);
	go2->boxes.push_back(goal->boxes[1]);
	if (checkRobots){
		go1->robots.push_back(goal->robots[0]);
		go2->robots.push_back(goal->robots[1]);
	}

	Search *s1 = new Search(st1, go1);
	copySearchParameters(s1);
	// s1.max_iterations = max_iterations;
	// s1.time_lim_secs = time_lim_secs;
	// s1.epsilon = epsilon;
	s1->search();
	// s1.print_plan();

	Search *s2 = new Search(st2, go2);
	copySearchParameters(s2);
	// s2.max_iterations = max_iterations;
	// s2.time_lim_secs = time_lim_secs;
	// s2.epsilon = epsilon;
	s2->search();
	// s2.print_plan();

	bool result = Search::paths_free({s1->plan,s2->plan});
	if (result){
		printf("INDV SOLUTIONS: ======================");
		s1->print_plan();
		s2->print_plan();
	}	
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

	return {stc, goc};

}




clashInfo Search::returnClashesStruct(vector<vector<State>> plans, int bt){
	// Takes plans as parameters, if the plans clash; returns a vector of
	// the state bt elements before the first clash, and the state bt elements
	// after the next one. If no clash, return empty vector
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

	State *st1 = &plan1[max+bt];
	State *go1 = &plan1[min-1];
	State *st2 = &plan2[max+bt];
	State *go2 = &plan2[min-1];

	printf("Clashing Starts: %s", plan1[max+bt].to_str().c_str());
	printf("Clashing Starts: %s", plan2[max+bt].to_str().c_str());

	printf("Clashing Ends: %s", plan1[min-1].to_str().c_str());
	printf("Clashing Ends: %s", plan2[min-1].to_str().c_str());

	clash.t_start = max+bt;
	clash.t_end = min-1;

	clash.clashes = {State::combine_states({st1,st2}), State::combine_states({go1,go2})};
	return clash;
}

void Search::copySearchParameters(Search* s){
	s->max_iterations = max_iterations;
	s->epsilon = epsilon;
	s->time_lim_secs = time_lim_secs;
}