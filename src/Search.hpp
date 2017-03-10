#ifndef SEARCH_H
#define SEARCH_H
#include <ctime>
#include <vector>
#include <stack>
#include <list>
#include "Open.hpp"
#include "Closed.hpp"

using std::string;
using std::vector;
using std::stack;
using std::list;

// Searchition structure
class Search{

	public:
		Search(State *startnode, State *goalnode);

		// Search results
		int num_exp_nodes;
		double planning_time;
	
		// Search parameters
		int max_iterations;
		float time_lim_secs;
		float epsilon;

		State *start;
		State *goal;

		// Plan
		vector<State> plan;

		// Loading configuration from file
		void load_search_parameters(char *filename);

		// Print plan
		void print_plan();

		// Store plan execution history
		void store_plan(char *filename);

		// Search for a plan
		int search();

		static bool paths_free(vector<vector<State>> plans);

	private:

		// Insert child to open vector if correct conditions met
		void new_child(State *child, Open *open, Closed *closed);
};

#endif
