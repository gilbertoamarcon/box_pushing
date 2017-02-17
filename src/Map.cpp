#include "Map.hpp"
#include "Pos.hpp"
#include "timeIt.cpp"
#include "utils.hpp"

Map::Map(char * filename){

	this->cols = 1;
	this->rows = 1;

	FILE *file;	
	char fileBuffer[BUFFER_SIZE]; 
	int aux = 0;

	// Checking if origin file exists
	file  = fopen(filename,"r");
	if(file == NULL){
		printf("Error: Origin file '%s' not found.\n",filename);
		return;
	}

	// Counting cols
	if(fgets(fileBuffer,BUFFER_SIZE,file) == NULL){
		printf("Error: Error while reading file.\n");
		return;
	}
	while(fileBuffer[aux] != '\n'){
		if(fileBuffer[aux] == ',')
			cols++;
		aux++;
	}

	// Counting rows
	while(fgets(fileBuffer,BUFFER_SIZE,file) != NULL) rows++;

	rewind(file);

	map = new int[cols*rows];

	// for(int i = rows-1; i >= 0; i--){
	for(int i = 0; i < rows; i++){

		// Load line
		fgets(fileBuffer,BUFFER_SIZE,file);

		aux = 0;
		for(int j = cols-1; j >= 0; j--){

			map[coordinate2index(i,j)] = atoi(fileBuffer+aux);

			// Seek next value
			while(fileBuffer[aux] != ',')
				if(fileBuffer[aux] == '\n')
					break;
				else
					aux++;
			aux++;
		}

	}

	fclose(file);

	printf("File '%s' loaded.\n",filename);

	return;

}

Map::~Map(){
	delete map;
};


int Map::coordinate2index(int i,int j){
	return i*cols+j;
}

void Map::index2coordinate(int index,int *i,int *j){
	*j = index%cols;
	*i = (index - (*j))/cols;
}

int Map::get_value(int i, int j){
	return map[coordinate2index(i,j)];
}

bool Map::is_Corner(int i, int j){
	if (!get_value(i, j)){
		if (get_value(i+1, j) && get_value(i, j+1))
			return true;
		else if (get_value(i+1, j) && get_value(i, j-1))
			return true;
		else if (get_value(i-1, j) && get_value(i, j+1))
			return true;
		else if (get_value(i-1, j) && get_value(i, j-1))
			return true;
		else
			return false;
	}
	return false;
}

void Map::set_Corners(){
	printf("Setting Corners\n");
	corners = {};
	for(int i=0; i<rows; i++)
		for(int j=0; j<cols; j++)
			if(is_Corner(i, j))
				corners.push_back(Pos(i, j));

	//for (int i=0;i<corners.size();i++)
	//	printf("%d %d \n", corners[i].i, corners[i].j);
}

void Map::set_Deadlocks(vector<Pos> goals){
	printf("Setting Deadlocks\n");
	deadlocks = {};
	for(int m=0; m<corners.size(); m++){
		for(int n=0; n<corners.size(); n++){
			if (!Pos::compare(corners[m], corners[n]) && n > m){
				if (evaluate_Corner_Pair(corners[m], corners[n], goals)){
					Pos c1 = corners[m];
					Pos c2 = corners[n];
					int r = c1.i;
					int c = c1.j;
					if(c1.i == c2.i){
						while (c != c2.j - sgn(c1.j - c2.j)){
							deadlocks.push_back(Pos(r,c));
							c -= sgn(c1.j - c2.j);
						}
					}
					else {
						while (r != c2.i - sgn(c1.i - c2.i)){
							deadlocks.push_back(Pos(r,c));
							r -= sgn(c1.i - c2.i);
							
						}
					}
				}
			}
		}
	}
	for (int i=0;i<corners.size();i++)
		if(!vec_contains(deadlocks, corners[i]) && !vec_contains(goals, corners[i]))
			deadlocks.push_back(corners[i]);

	//Print all deadlocks
	//for (int i=0;i<deadlocks.size();i++)
	//	printf("%d %d \n", deadlocks[i].i, deadlocks[i].j);

	//DISABLE
	deadlocks = {};

}


bool Map::evaluate_Corner_Pair(Pos c1, Pos c2, vector<Pos> goals){
	if(c1.i == c2.i){
		bool l_wall = true;
		bool r_wall = true;
		int x = c1.i;
		int y = c1.j;
		while (y != c2.j){
			y -= sgn(c1.j - c2.j);
			if (get_value(x, y) || vec_contains(goals, Pos(x,y)))
				return false;
			if (!get_value(x+1, y))
				r_wall = false;
			if (!get_value(x-1, y))
				l_wall = false;
		}
		if(r_wall || l_wall)
			return true;
	}
	else if(c1.j == c2.j){
		bool u_wall = true;
		bool d_wall = true;
		int x = c1.i;
		int y = c1.j;
		while (x != c2.i){
			x -= sgn(c1.i - c2.i);
			if (get_value(x, y) || vec_contains(goals, Pos(x,y)))
				return false;
			if (!get_value(x, y+1))
				u_wall = false;
			if (!get_value(x, y-1))
				d_wall = false;
		}
		if(u_wall || d_wall)
			return true;
	}
	return false;
}
