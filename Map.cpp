#include "Map.hpp"

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
