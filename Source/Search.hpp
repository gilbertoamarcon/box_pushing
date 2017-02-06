#ifndef SEARCH_H
#define SEARCH_H
#include <ctime>
#include <string>
#include <vector>
#include <stack>
#include <list>
#include "State.hpp"

// File names
#define PROB_FILE	"Files/problem.csv"
#define MAP_FILE	"Files/map.csv"
#define PLAN_FILE	"Files/plan.csv"
#define CFG_FILE	"Files/cfg.csv"

using std::string;
using std::vector;
using std::stack;
using std::list;

// Searchition structure
class Search{

	public:
	
		// Search results
		static int num_exp_nodes;
		static double planning_time;
	
		// Search parameters
		static int max_iterations;
		static int beamsize;
		static float epsilon;

		// Plan
		static stack<State> plan;

		// Loading configuration from file
		static void load_search_parameters(char *filename);

		// Print plan
		static void print_plan();

		// Store plan execution history
		static void store_plan(char *filename);

		// Search for a plan
		static void search();

	private:

		// Insert child to open list if correct conditions met
		static void new_child(State *child, list<State*> *open, vector<State*> *closed);
};

#endif
