#ifndef ASTAR_HPP_INCLUDED
#define ASTAR_HPP_INCLUDED

#include <utility>
#include <cmath>
#include <limits>
#include <vector>
#include <set>
#include <stack>
#include <iostream>

#define INF std::numeric_limits<float>::max()

using pair_type = std::pair<int,int>;
using dpair_type = std::pair<double, pair_type>;
using grid_type = std::vector<int>;

struct Cell {
	int row, col;
	double f, g, h;
};

bool isValid(const pair_type & point, const int & width, const int & height) {
	return (point.first >= 0) && (point.first < height) && (point.second >= 0) && (point.second < width);
}

bool isBlocked(const grid_type & grid, int index) {
	if (grid[index] == 0) return true;
	return false;
}

bool isDestination(const pair_type & point, const pair_type & dest) {
	if (point == dest) return true;
	return false;
}

double calculateHeuristic(const pair_type & point, const pair_type & dest) {
	return std::sqrt((point.first - dest.first) * (point.first - dest.first) + (point.second - dest.second) * (point.second - dest.second));
}

void trace(const std::vector<Cell> & cellDetails, const pair_type & dest, const int & width) {
	std::cout << "The Path is";
	
	std::stack<pair_type> path;
	auto row = dest.first;
	auto col = dest.second;
	
	while (!(cellDetails[row*width+col].row == row && cellDetails[row*width+col].col == col)) {
		path.push(std::make_pair(row, col));
		auto nextRow = cellDetails[row*width+col].row;
		auto nextCol = cellDetails[row*width+col].col;
		row = nextRow; col = nextCol;
	}
	
	path.push(std::make_pair(row, col));
	
	while (!path.empty()) {
		auto point = path.top(); path.pop();
		std::cout << " -> (" << point.first << "," << point.second << ")";
	}
	
	std::cout << std::endl;
}

bool checkAdjCell(const grid_type & grid, const pair_type & point, const pair_type & adj, const int & index, const int & width, const int & height, const pair_type & dest, const std::vector<bool> & closedList, std::set<dpair_type> & openList, std::vector<Cell> & cellDetails) {
	auto tempIdx = adj.first * width + adj.second;
	
	if (isValid(adj, width, height)) {
		if (isDestination(adj, dest)) {
			cellDetails[tempIdx].row = point.first;
			cellDetails[tempIdx].col = point.first;
			return true;
		}
		else if (!closedList[tempIdx] && !isBlocked(grid, tempIdx)) {
			double g = cellDetails[index].g;
			if (point.first != adj.first && point.second != adj.second) g += 1.414;
			else g += 1.0;
			double h = calculateHeuristic(adj, dest);
			double f = g + h;
			
			if (cellDetails[tempIdx].f == INF || cellDetails[tempIdx].f > f) {
				openList.insert(std::make_pair(f, adj));
				cellDetails[tempIdx].f = f;
				cellDetails[tempIdx].g = g;
				cellDetails[tempIdx].h = h;
				cellDetails[tempIdx].row = point.first;
				cellDetails[tempIdx].col = point.second;
			}
		}
	}
	
	return false;
}

void aStar(const grid_type & grid, const pair_type & start, const pair_type & end, const int & width, const int & height) {
	if (!isValid(start, width, height)) return;
	if (!isValid(end, width, height)) return;
	
	auto startIdx = start.first * width + start.second;
	auto endIdx = end.first * width + end.second;
	
	if (isBlocked(grid, startIdx) || isBlocked(grid, endIdx)) return;
	if (isDestination(start, end)) return;
	
	std::vector<bool> closedList(width*height, false);
	std::vector<Cell> cellDetails(width*height, {-1, -1, INF, INF, INF});
	
	cellDetails[startIdx].f = 0.0;
	cellDetails[startIdx].g = 0.0;
	cellDetails[startIdx].h = 0.0;
	cellDetails[startIdx].row = start.first;
	cellDetails[startIdx].col = start.second;
	
	std::set<dpair_type> openList;
	openList.insert(std::make_pair(0.0, start));
	
	bool endFound = false;
	
	while (!openList.empty()) {
		auto temp = *openList.begin();
		
		openList.erase(openList.begin());
		
		auto point = temp.second;
		auto index = point.first * width + point.second;
		
		closedList[index] = true;
		
		if (!endFound) endFound = checkAdjCell(grid, point, std::make_pair(point.first-1, point.second), index, width, height, end, closedList, openList, cellDetails);
		else break;
		
		if (!endFound) endFound = checkAdjCell(grid, point, std::make_pair(point.first+1, point.second), index, width, height, end, closedList, openList, cellDetails);
		else break;
		
		if (!endFound) endFound = checkAdjCell(grid, point, std::make_pair(point.first, point.second+1), index, width, height, end, closedList,  openList, cellDetails);
		else break;
		
		if (!endFound) endFound = checkAdjCell(grid, point, std::make_pair(point.first, point.second-1), index, width, height, end, closedList,  openList, cellDetails);
		else break;
		
		if (!endFound) endFound = checkAdjCell(grid, point, std::make_pair(point.first-1, point.second+1), index, width, height, end, closedList,  openList, cellDetails);
		else break;
		
		if (!endFound) endFound = checkAdjCell(grid, point, std::make_pair(point.first-1, point.second-1), index, width, height, end, closedList, openList, cellDetails);
		else break;
		
		if (!endFound) endFound = checkAdjCell(grid, point, std::make_pair(point.first+1, point.second+1), index, width, height, end, closedList,  openList, cellDetails);
		else break;
		
		if (!endFound) endFound = checkAdjCell(grid, point, std::make_pair(point.first+1, point.second-1), index, width, height, end, closedList,  openList, cellDetails);
		else break;
		
		if (endFound) break;
	}
	
	if (endFound) trace(cellDetails, end, width);
	else std::cout << "No path was found" << std::endl; 
}

#endif