#ifndef MAP_H
#define MAP_H
#include <stdlib.h>
#include <stdio.h>
#include <cstdlib>

using namespace std;

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
		Map(char * filename);
		virtual ~Map();
		int get_value(int i, int j);
};

#endif
