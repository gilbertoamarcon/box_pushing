#include "Search.hpp"
#include "timeIt.hpp"
// File names
#define CFG_FILE	"files/cfg.csv"
#define PROBLEM_PRE	"files/problem"
#define MAP_FILE	"/map.csv"
#define PROB_FILE	"/problem.csv"
#define PLAN_FILE_FC	"/plan.csv"
#define PLAN_FILE_MD	"/plan_Mdecouple.csv"

int main(int argc, char **argv){

	// Line arguments
	int result		= -1;
	int verbose		= 0;
	int problem_number = 33;
	for(int i = 1; i < argc; i++){
		if(argv[i][0] == '-' && argv[i][1] == 'v') verbose	= 1; else
		problem_number = atoi(argv[i]);
	}

	// File names
	char map_file[BUFFER_SIZE];
	char prob_file[BUFFER_SIZE];
	char plan_file_fc[BUFFER_SIZE];
	char plan_file_md[BUFFER_SIZE];
	sprintf(map_file,"%s%d%s",PROBLEM_PRE,problem_number,MAP_FILE);
	sprintf(prob_file,"%s%d%s",PROBLEM_PRE,problem_number,PROB_FILE);
	sprintf(plan_file_md,"%s%d%s",PROBLEM_PRE,problem_number,PLAN_FILE_MD);
	sprintf(plan_file_fc,"%s%d%s",PROBLEM_PRE,problem_number,PLAN_FILE_FC);

	State::load_map(map_file);
	State::load_problem(prob_file);
	State::display_world(State::start);

	tic();
	Search s(State::start, State::goal);
	s.load_search_parameters(CFG_FILE);
	s.searchDecoupled1();
	s.print_plan();
	s.store_plan(plan_file_md);
	printf("%d expanded nodes.\n",s.num_exp_nodes);
	toc();

	// tic();
	// Search c(State::start, State::goal);
	// sc.load_search_parameters(CFG_FILE);
	// sc.search();
	// sc.print_plan();
	// sc.store_plan(plan_file_fc);
	// printf("%d expanded nodes.\n",sc.num_exp_nodes);
	// toc();

	return 0;
}