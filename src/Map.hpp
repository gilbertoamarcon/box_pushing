#ifndef MAP_H
#define MAP_H
#include <cstdio>
#include <cstdlib>
#include <vector>
#include "Pos.hpp"

using std::vector;

// File Parameters
#define BUFFER_SIZE		256

class Map{

	private:
		int coordinate2index(int i,int j);
		void index2coordinate(int index,int *i,int *j);

	public:
		int cols;
		int rows;
		int *map;
		vector<Pos> corners;
		vector<Pos> deadlocks;
		Map(char * filename);
		virtual ~Map();
		int get_value(int i, int j);
		bool is_Corner(int i, int j);
		bool is_Deadlock(int i, int j);
		bool evaluate_Corner_Pair(Pos c1, Pos c2, vector<Pos> goals);
		void set_Corners();
		void set_Deadlocks(vector<Pos> goals);


};

#endif
