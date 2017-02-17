#include "Search.hpp"

// File names
#define CFG_FILE	"files/cfg.csv"
#define MAP_FILE	"files/ovuncTest/map.csv"
#define PROB_FILE	"files/ovuncTest/problem.csv"
#define PLAN_FILE	"files/ovuncTest/plan.csv"

int main(int argc, char **argv){
	// Search configuration parameters 
	

	// Loading obstacle map
	State::load_map(MAP_FILE);

	// Loading problem
	State::load_problem(PROB_FILE);
	//printf("Start: %s",	State::start->to_str());
	//printf("Goal: %s",	State::goal->to_str());
	State::display_world(State::start);


	//State starttest("1,1:2,2\n");
	//State goaltest("3,3");
	//State *starttest = new State("2,2,2,6:3,7");
	//State *goaltest = new State("3,3:1,1");

	//printf("\n\n%d\n\n", starttest.boxes [0].i);
	//Search::search_ind(&starttest, &goaltest);

	// Running and taking execution time
	//Search::search();

	State starttest({Pos(1,4)},{Pos(1,5)});
	State goaltest({Pos(1,3)},{Pos(1,4)});
	Search searchtest(&starttest, &goaltest);
	searchtest.load_search_parameters(CFG_FILE);
	searchtest.search();
	//Presenting results on screen


	if(searchtest.num_exp_nodes > 0){
		printf("Plan found.\n");
		printf("%d expanded nodes.\n",searchtest.num_exp_nodes);
		printf("%d actions.\n",searchtest.plan.size());
		printf("%f seconds.\n",searchtest.planning_time);
		searchtest.print_plan();
		searchtest.store_plan(PLAN_FILE);
	}
	else
		printf("Plan failed.\n");
		

	return 0;

}