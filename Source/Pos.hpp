#ifndef POS_H
#define POS_H
#include <stdlib.h>
#include <stdio.h>
#include <cstdlib>
#include <string>
#include <cmath>
#include <vector>
#include <stack>

// File Parameters
#define BUFFER_SIZE		256

using namespace std;

// Position structure
class Pos{

	public:

		int i;
		int j;

		// Parsing string into position vector
		static void parse(char *str,vector<Pos> *pos);

		// Compare two position vectors
		//  1: sta > state
		//  0: sta = state
		// -1: sta < state
		static int compare_vec(vector<Pos> &veca,vector<Pos> &vecb);

		// Manhattan distance between pos a and pos b
		static int manhattan(Pos a, Pos b);

		// Compares two positions
		static bool compare(Pos a, Pos b);
		
		// Position constructor
		Pos(int i, int j){
			this->i = i;
			this->j = j;
		}

		// Return string representation of position
		char* sprint();
};

#endif
