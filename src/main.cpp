#include "Search.hpp"

// File names
#define CFG_FILE	"files/cfg.csv"
#define MAP_FILE	"files/ovuncTest/map.csv"
#define PROB_FILE	"files/ovuncTest/problem.csv"
#define PLAN_FILE	"files/ovuncTest/plan.csv"

int main(int argc, char **argv){
	// Search configuration parameters 
	Search::load_search_parameters(CFG_FILE);

	// Loading obstacle map
	State::load_map(MAP_FILE);

	// Loading problem
	State::load_problem(PROB_FILE);
	printf("Start: %s",	State::start->to_str());
	printf("Goal: %s",	State::goal->to_str());
	State::display_world(State::start);

	State starttest({Pos(1,1)},{Pos(2,2)});
	State goaltest({Pos(2,2)},{Pos(1,1)});
	//State starttest("1,1:2,2\n");
	//State goaltest("3,3");
	//State *starttest = new State("2,2,2,6:3,7");
	//State *goaltest = new State("3,3:1,1");

	//printf("\n\n%d\n\n", starttest.boxes [0].i);
	Search::search_ind(&starttest, &goaltest);

	// Running and taking execution time
	//Search::search();

	//Presenting results on screen
	if(Search::num_exp_nodes > 0){
		printf("Plan found.\n");
		printf("%d expanded nodes.\n",Search::num_exp_nodes);
		printf("%d actions.\n",Search::plan.size());
		printf("%f seconds.\n",Search::planning_time);
		Search::print_plan();
		Search::store_plan(PLAN_FILE);
	}
	else
		printf("Plan failed.\n");
		

	return 0;

}