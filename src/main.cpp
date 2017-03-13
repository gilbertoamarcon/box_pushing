#include "Search.hpp"

// File names
#define CFG_FILE	"files/cfg.csv"
#define PROBLEM_PRE	"files/problem"
#define MAP_FILE	"/map.csv"
#define PROB_FILE	"/problem.csv"
#define PLAN_FILE	"/plan.csv"

int main(int argc, char **argv){

	// Line arguments
	int result		= -1;
	int verbose		= 0;
	int problem_number = 26;
	for(int i = 1; i < argc; i++){
		if(argv[i][0] == '-' && argv[i][1] == 'v') verbose	= 1; else
		problem_number = atoi(argv[i]);
	}

	// File names
	char map_file[BUFFER_SIZE];
	char prob_file[BUFFER_SIZE];
	char plan_file[BUFFER_SIZE];
	sprintf(map_file,"%s%d%s",PROBLEM_PRE,problem_number,MAP_FILE);
	sprintf(prob_file,"%s%d%s",PROBLEM_PRE,problem_number,PROB_FILE);
	sprintf(plan_file,"%s%d%s",PROBLEM_PRE,problem_number,PLAN_FILE);

	// Loading obstacle map
	State::load_map(map_file);

	// Loading problem
	State::load_problem(prob_file);

	Search search1(State::start, State::goal);
	// Search configuration parameters
	search1.load_search_parameters(CFG_FILE);


	if(verbose){
		printf("Start: %s",	State::start->to_str().c_str());
		printf("Goal: %s",	State::goal->to_str().c_str());
		State::display_world(State::start);
	}

	// Running and taking execution time
	result = search1.search();

	if(search1.num_exp_nodes > 0)
		search1.store_plan(plan_file);

	// Presenting results on screen	
	if(!verbose) return result;
	if(search1.num_exp_nodes > 0){
			printf("Plan found.\n");
			printf("%d expanded nodes.\n",search1.num_exp_nodes);
			printf("%d actions.\n",search1.plan.size());
			printf("%f seconds.\n",search1.planning_time);
			search1.print_plan();
	}
	else
		printf("Plan failed.\n");

	return result;

}