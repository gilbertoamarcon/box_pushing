#ifndef SEARCH_H
#define SEARCH_H
#include <ctime>
#include <vector>
#include <stack>
#include <list>
#include "State.hpp"
#include "Open.hpp"
#include "Closed.hpp"

using std::string;
using std::vector;
using std::stack;
using std::list;


struct clashInfo
{
	bool isClashing;
    int t_start;
    int t_end;
    vector<State*> clashes;
};

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

		bool checkRobots = false;
		bool coupled = false;

		vector<vector<State>> aux_plan_vec;

		State *start;
		State *goal;
		State *reducedGoal;



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

		void searchDecoupled1();

		void searchDecoupled2();

		bool checkReducedSolution(State* cur);

		void copySearchParameters(Search* s);

		static void print_plan(vector<State> p);

		static bool paths_free(vector<vector<State>> plans);

		static vector<State*> returnClashes(vector<vector<State>> plans);

		static clashInfo returnClashesStruct(vector<vector<State>> plans, int bt);

		static vector<State> mergePlans(Search* s1, Search* s2, Search* sc, clashInfo c);

		static vector<State> mergePlans2(Search* s1, Search* s2, Search* sc, clashInfo c);
	private:

		// Insert child to open vector if correct conditions met
		void new_child(State *child, Open *open, Closed *closed);
};

#endif
