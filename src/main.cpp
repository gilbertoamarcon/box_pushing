#include "Search.hpp"

// File names
#define CFG_FILE	"files/cfg.csv"
#define PROBLEM_PRE	"files/problem"
#define MAP_FILE	"/map.csv"
#define PROB_FILE	"/problem.csv"
#define PLAN_FILE	"/plan.csv"

int main(int argc, char **argv){

	// Line arguments
	char map_file[BUFFER_SIZE];
	char prob_file[BUFFER_SIZE];
	char plan_file[BUFFER_SIZE];
	int problem_number = 1;
	if(argc >= 2)
		problem_number = atoi(argv[1]);
	sprintf(map_file,"%s%d%s",PROBLEM_PRE,problem_number,MAP_FILE);
	sprintf(prob_file,"%s%d%s",PROBLEM_PRE,problem_number,PROB_FILE);
	sprintf(plan_file,"%s%d%s",PROBLEM_PRE,problem_number,PLAN_FILE);
		
	// Search configuration parameters
	Search::load_search_parameters(CFG_FILE);

	// Loading obstacle map
	State::load_map(map_file);

	// Loading problem
	State::load_problem(prob_file);
	printf("Start: %s",	State::start->to_str().c_str());
	printf("Goal: %s",	State::goal->to_str().c_str());
	State::display_world(State::start);

	// Running and taking execution time
	Search::search();

	// Presenting results on screen
	if(Search::num_exp_nodes > 0){
		printf("Plan found.\n");
		printf("%d expanded nodes.\n",Search::num_exp_nodes);
		printf("%d actions.\n",Search::plan.size());
		printf("%f seconds.\n",Search::planning_time);
		Search::print_plan();
		Search::store_plan(plan_file);
	}
	else
		printf("Plan failed.\n");

	return 0;

}