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
		


	// //State start1({Pos(2,2)},{Pos(1,1)});  // BoxPos, AgentPos
	// //State goal1({Pos(2,3)},{Pos(1,1)});

	// State *start1 = new State({Pos(2,2), Pos(6,2)},{Pos(1,1), Pos(7,1)});
	// State *goal1 = new State({Pos(2,6), Pos(6,6)},{});

	// // Search configuration parameters
	
	State::load_map(map_file);
	State::load_problem(prob_file);

	// Search search1(start1, goal1);
	// // Search search1(State::start, State::goal);
	// search1.load_search_parameters(CFG_FILE);

	// search1.search();
	// search1.print_plan();
	
	// tic();
	// State *st = new State({Pos(6,2),Pos(2,2)},{Pos(7,1),Pos(1,1)});
	// State *go = new State({Pos(2,6),Pos(6,6)},{});
	// Search s(st, go);
	// s.load_search_parameters(CFG_FILE);
	// s.search();
	// s.print_plan();
	// toc();

	State *std = new State({Pos(6,2),Pos(2,2)},{Pos(7,1),Pos(1,1)});
	State *god = new State({Pos(2,6),Pos(6,6)},{});
	//SPLICE SEARCH
	tic();
	printf("Distributing Tasks...\n");

	State *st1 = new State({Pos(6,2)},{Pos(7,1)});
	State *go1 = new State({Pos(2,6)},{});
	State *st2 = new State({Pos(2,2)},{Pos(1,1)});
	State *go2 = new State({Pos(6,6)},{});
	toc();
	//SEARCH INDIVIDUALLY
	tic();
	printf("Searching Individually...\n");

	Search s1(st1, go1);
	s1.load_search_parameters(CFG_FILE);
	s1.search();

	Search s2(st2, go2);
	s2.load_search_parameters(CFG_FILE);
	s2.search();
	toc();
	//CHECK COLLISION SET
	tic();
	printf("Checking Collision Set...\n");

	clashInfo c = Search::returnClashesStruct({s1.plan,s2.plan});
	printf("CLASHING START_END: %d %d \n",c.t_start,c.t_end);

	toc();
	//DO COUPLED SOLUTION IF CLASHING
	tic();
	if (c.clashes.size() == 0)
		printf("No clashes between individual plans, executing paths...\n");
	else
		printf("Doing local coupled search...\n");
		Search sc(c.clashes[0], c.clashes[1]);
		sc.load_search_parameters(CFG_FILE);
		sc.checkRobots = true;
		sc.coupled = true;
		sc.search();
	toc();

	//TIE ALL THE RESULTING PATHS
	tic();	
	printf("Displaying Final Path...\n");
	printf("%d %d %d %d",s1.plan.begin(),s1.plan.begin(),c.t_start,c.t_end);
	vector<State> p1(s1.plan.begin() + c.t_start, s1.plan.end()); 
	std::reverse(p1.begin(), p1.end());
	vector<State> p2(s2.plan.begin() + c.t_start, s2.plan.end()); 	
	std::reverse(p2.begin(), p2.end());
	vector<State> p3(s1.plan.end() - c.t_end, s1.plan.end());
	std::reverse(p3.begin(), p3.end()); 
	vector<State> p4(s2.plan.end() - c.t_end, s2.plan.end()); 
	std::reverse(p4.begin(), p4.end());
	vector<State> p5(sc.plan);
	std::reverse(p5.begin(), p5.end());
	toc();

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

	printf("Merged INDV PATHS1...\n");
	//MERGE INDV PATHS
	int max1 = std::max(p1.size(), p2.size());
	vector<State> pp1 = {};
	for (int i=0;i<max1;i++)
		pp1.push_back(*State::combine_states({&(p1[i]),&(p2[i])}));

	for (int i=0;i<pp1.size();i++)
		printf("%s \n",pp1[i].to_str().c_str());

	printf("Merged INDV PATHS2...\n");
	//MERGE INDV PATHS
	int max2 = std::max(p3.size(), p4.size());
	vector<State> pp2 = {};
	for (int i=0;i<max2;i++)
		pp1.push_back(*State::combine_states({&(p3[i]),&(p4[i])}));

	for (int i=0;i<pp2.size();i++)
		printf("%s \n",pp2[i].to_str().c_str());

	//ACTUAL MERGING
	//pp1+p5+pp2
	pp1.insert(pp1.end(), p5.begin(), p5.end()); 
	pp1.insert(pp1.end(), pp2.begin(), pp2.end()); 
	printf("FINAL PATH...\n");
	for (int i=0;i<pp1.size();i++)
		printf("%s \n",pp1[i].to_str().c_str());

	std::reverse(pp1.begin(), pp1.end());
	Search::print_plan(pp1);



	// tic();

	// State *st1 = new State({Pos(6,2)},{Pos(7,1)});
	// State *go1 = new State({Pos(2,6)},{});
	// State *st2 = new State({Pos(2,2)},{Pos(1,1)});
	// State *go2 = new State({Pos(6,6)},{});

	// Search s1(st1, go1);
	// s1.load_search_parameters(CFG_FILE);
	// s1.search();
	// s1.print_plan();

	// Search s2(st2, go2);
	// s2.load_search_parameters(CFG_FILE);
	// s2.search();
	// s2.print_plan();

	// toc();

	// State *start1 = new State({Pos(5,6), Pos(3,6)},{Pos(6,6), Pos(2,6)});  // BoxPos, AgentPos
	// State *goal1 = new State({Pos(2,6), Pos(6,6)},{Pos(3,6), Pos(5,6)});
	// Search s3(start1, goal1);
	// s3.load_search_parameters(CFG_FILE);
	// s3.search();
	// s3.print_plan();

	// vector<State*> clashes = Search::returnClashes({s1.plan,s2.plan});
	// printf((*clashes[0]).to_str().c_str());
	// printf((*clashes[1]).to_str().c_str());

	// if (clashes.size() == 0)
	// 	printf("No clashes between individual plans, executing paths...");
	// else
	// 	printf("Doing local coupled search...");
	// 	Search sc(clashes[1], clashes[0]);
	// 	tic();
	// 	sc.load_search_parameters(CFG_FILE);
	// 	sc.checkRobots = true;
	// 	sc.coupled = true;
	// 	sc.search();
	// 	sc.print_plan();

	// toc();

	// delete clashes[0];
	// delete clashes[1];

	// delete &s1;
	// delete &s2;


	// // Loading obstacle map
	// State::load_map(map_file);

	// // Loading problem
	// State::load_problem(prob_file);
	// if(verbose){
	// 	printf("Start: %s",	State::start->to_str().c_str());
	// 	printf("Goal: %s",	State::goal->to_str().c_str());
	// 	State::display_world(State::start);
	// }

	// // Running and taking execution time
	// result = search1.search();

	// if(search1.num_exp_nodes > 0)
	// 	search1.store_plan(plan_file);

	// // Presenting results on screen
	// if(!verbose) return result;
	// if(search1.num_exp_nodes > 0){
	// 		printf("Plan found.\n");
	// 		printf("%d expanded nodes.\n",search1.num_exp_nodes);
	// 		printf("%d actions.\n",search1.plan.size());
	// 		printf("%f seconds.\n",search1.planning_time);
	// 		search1.print_plan();
	// }
	// else
	// 	printf("Plan failed.\n");

	return result;

}