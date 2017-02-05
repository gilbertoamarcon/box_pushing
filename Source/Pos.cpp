#include "State.hpp"

// Parsing string into position vector
void Pos::parse(char *str, vector<Pos> *pos){
	int i = 0;
	int j = 0;
	for(;;){
		i = atoi(str);
		while(str[0] != ',') str++; str++;
		j = atoi(str);
		pos->push_back(Pos(i,j));
		while(str[0] != ',' && str[0] != '\0') str++;
		if(str[0] == '\0') return;
		str++;
	}
}

// Compare two position vectors
//  1: sta > state
//  0: sta = state
// -1: sta < state
int Pos::compare_vec(vector<Pos> &veca,vector<Pos> &vecb){
	if(veca.size() > vecb.size()) return  1;
	if(veca.size() < vecb.size()) return -1;
	for(int i = 0; i < veca.size(); i++){
		if(veca.at(i).i > vecb.at(i).i) return  1;
		if(veca.at(i).i < vecb.at(i).i) return -1;
		if(veca.at(i).j > vecb.at(i).j) return  1;
		if(veca.at(i).j < vecb.at(i).j) return -1;
	}
	return 0;
}

// Manhattan distance between a and b
int Pos::manhattan(Pos a, Pos b){
	return abs(a.i - b.i) + abs(a.j - b.j);
}

// Compares two positions
bool Pos::compare(Pos a, Pos b){
	return a.i == b.i && a.j == b.j;
}

// Return string representation of position
void Pos::sprint(char *buffer){
	sprintf(buffer,"%s%d,%d,",buffer,i,j);
}