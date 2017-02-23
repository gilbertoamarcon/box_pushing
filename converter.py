#!/usr/bin/python
import os

problem_num = 0;
for in_file_name in sorted(os.listdir('files/sokoban-strips')):
	problem_num +=1;
	map_file_name = "files/problem"+str(problem_num)+"/map.csv";
	prob_file_name = "files/problem"+str(problem_num)+"/problem.csv";
	if not os.path.exists("files/problem"+str(problem_num)):
		os.mkdir("files/problem"+str(problem_num));
	if not os.path.exists("files/problem"+str(problem_num)+"/figs"):
		os.mkdir("files/problem"+str(problem_num)+"/figs");
	in_file = open('files/sokoban-strips/'+in_file_name);
	map_file = open(map_file_name, "w");
	prob_file = open(prob_file_name, "w");
	i = 0;
	j = 0;
	agts = [];
	boxs = [];
	dest = [];

	width  = 0;
	for line in in_file:
		if line.find(';; ') != -1:
			string = line[3:];
			if len(string) >= width:
				width = len(string);
	in_file.close();

	in_file = open('files/sokoban-strips/'+in_file_name);
	for line in in_file:
		if line.find(';; ') != -1:
			string = line[3:];
			str_i = 0;
			while str_i < len(string)-1:
				if string[str_i] == "@":
					agts.append([i, j]);
				if string[str_i] == "$":
					boxs.append([i, j]);
				if string[str_i] == ".":
					dest.append([i, j]);
				if string[str_i] == "#":
					map_file.write('1');
				if str_i+1 != len(string)-1:
					map_file.write(',');
				str_i += 1;
				j = j+1;
			while j < width-1:
				map_file.write(',');
				j = j+1;
			map_file.write('\n');
			i += 1;
			j = 0;

	for box in boxs:
		prob_file.write(str(box[0])+','+str(box[1])+',');
	prob_file.write(':');
	for agt in agts:
		prob_file.write(str(agt[0])+','+str(agt[1])+',');
	prob_file.write('\n');
	for des in dest:
		prob_file.write(str(des[0])+','+str(des[1])+',');
	prob_file.write(':\n');

	in_file.close();
	map_file.close();
	prob_file.close();