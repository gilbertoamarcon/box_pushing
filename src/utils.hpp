#ifndef UTILS_H
#define UTILS_H
#include <vector>
#include "Pos.hpp"
#include <algorithm>
//Misc Utility Functions

template <typename T> inline int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

inline bool vec_contains(vector<Pos> v, Pos p) {
	for (int k=0; k<v.size(); k++)
		if (v[k].i == p.i && v[k].j == p.j)
			return true;
	return false;
}


template <typename T> inline int get_index(vector<T> vec, T element) {
	return std::find(vec.begin(), vec.end(), element) - vec.begin();
}

#endif