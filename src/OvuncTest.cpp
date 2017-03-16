#include "Search.hpp"
#include "timeIt.hpp"
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

	State::load_map(map_file);
	State::load_problem(prob_file);
	State::display_world(State::start);
	State::display_world(State::goal);

	// tic();
	// Search s(State::start, State::goal);
	// s.load_search_parameters(CFG_FILE);
	// s.searchDecoupled2();
	// s.print_plan();
	// s.store_plan(plan_file);
	// printf("%d expanded nodes.\n",s.num_exp_nodes);
	// toc();


	tic();
	Search sc(State::start, State::goal);
	sc.load_search_parameters(CFG_FILE);
	sc.search();
	sc.print_plan();
	sc.store_plan(plan_file);
	printf("%d expanded nodes.\n",sc.num_exp_nodes);
	toc();

	return 0;
}