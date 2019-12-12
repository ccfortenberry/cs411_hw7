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
		//std::cout << "\nBuilding point (" << row << "," << col << ")\n";
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
			cellDetails[tempIdx].col = point.second;
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
		
		//std::cout << "\nlooking at point (" << point.first << "," << point.second << ")\n";
		
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
		
		/* auto north = std::make_pair(point.first-1, point.second);
		auto south = std::make_pair(point.first+1, point.second);
		auto east = std::make_pair(point.first, point.second+1);
		auto west = std::make_pair(point.first, point.second-1);
		auto northeast = std::make_pair(point.first-1, point.second+1);
		auto northwest = std::make_pair(point.first-1, point.second-1);
		auto southeast = std::make_pair(point.first+1, point.second+1);
		auto southwest = std::make_pair(point.first+1, point.second-1);
		
		if (isValid(north, width, height)) {
			auto northIdx = north.first * width + north.second;
			if (isDestination(north, end)) {
				cellDetails[northIdx].row = point.first;
				cellDetails[northIdx].col = point.second;
				endFound = true;
				break;
			}
			else if (!closedList[northIdx] && !isBlocked(grid, northIdx)) {
				double g = cellDetails[index].g + 1.0;
				double h = calculateHeuristic(north, end);
				double f = g + h;
				
				if (cellDetails[northIdx].f == INF || cellDetails[northIdx].f > f) {
					openList.insert(std::make_pair(f, north));
					cellDetails[northIdx].f = f;
					cellDetails[northIdx].g = g;
					cellDetails[northIdx].h = h;
					cellDetails[northIdx].row = point.first;
					cellDetails[northIdx].col = point.second;
				}
			}
		}
		
		if (isValid(south, width, height)) {
			auto southIdx = south.first * width + south.second;
			if (isDestination(south, end)) {
				cellDetails[southIdx].row = point.first;
				cellDetails[southIdx].col = point.second;
				endFound = true;
				break;
			}
			else if (!closedList[southIdx] && !isBlocked(grid, southIdx)) {
				double g = cellDetails[index].g + 1.0;
				double h = calculateHeuristic(south, end);
				double f = g + h;
				
				if (cellDetails[southIdx].f == INF || cellDetails[southIdx].f > f) {
					openList.insert(std::make_pair(f, south));
					cellDetails[southIdx].f = f;
					cellDetails[southIdx].g = g;
					cellDetails[southIdx].h = h;
					cellDetails[southIdx].row = point.first;
					cellDetails[southIdx].col = point.second;
				}
			}
		}
		
		if (isValid(east, width, height)) {
			auto eastIdx = east.first * width + east.second;
			if (isDestination(east, end)) {
				cellDetails[eastIdx].row = point.first;
				cellDetails[eastIdx].col = point.second;
				endFound = true;
				break;
			}
			else if (!closedList[eastIdx] && !isBlocked(grid, eastIdx)) {
				double g = cellDetails[index].g + 1.0;
				double h = calculateHeuristic(east, end);
				double f = g + h;
				
				if (cellDetails[eastIdx].f == INF || cellDetails[eastIdx].f > f) {
					openList.insert(std::make_pair(f, east));
					cellDetails[eastIdx].f = f;
					cellDetails[eastIdx].g = g;
					cellDetails[eastIdx].h = h;
					cellDetails[eastIdx].row = point.first;
					cellDetails[eastIdx].col = point.second;
				}
			}
		}
		
		if (isValid(west, width, height)) {
			auto westIdx = west.first * width + west.second;
			if (isDestination(west, end)) {
				cellDetails[westIdx].row = point.first;
				cellDetails[westIdx].col = point.second;
				endFound = true;
				break;
			}
			else if (!closedList[westIdx] && !isBlocked(grid, westIdx)) {
				double g = cellDetails[index].g + 1.0;
				double h = calculateHeuristic(west, end);
				double f = g + h;
				
				if (cellDetails[westIdx].f == INF || cellDetails[westIdx].f > f) {
					openList.insert(std::make_pair(f, west));
					cellDetails[westIdx].f = f;
					cellDetails[westIdx].g = g;
					cellDetails[westIdx].h = h;
					cellDetails[westIdx].row = point.first;
					cellDetails[westIdx].col = point.second;
				}
			}
		}
		
		if (isValid(northeast, width, height)) {
			auto northeastIdx = northeast.first * width + northeast.second;
			if (isDestination(northeast, end)) {
				cellDetails[northeastIdx].row = point.first;
				cellDetails[northeastIdx].col = point.second;
				endFound = true;
				break;
			}
			else if (!closedList[northeastIdx] && !isBlocked(grid, northeastIdx)) {
				double g = cellDetails[index].g + 1.414;
				double h = calculateHeuristic(northeast, end);
				double f = g + h;
				
				if (cellDetails[northeastIdx].f == INF || cellDetails[northeastIdx].f > f) {
					openList.insert(std::make_pair(f, northeast));
					cellDetails[northeastIdx].f = f;
					cellDetails[northeastIdx].g = g;
					cellDetails[northeastIdx].h = h;
					cellDetails[northeastIdx].row = point.first;
					cellDetails[northeastIdx].col = point.second;
				}
			}
		}
		
		if (isValid(northwest, width, height)) {
			auto northwestIdx = northwest.first * width + northwest.second;
			if (isDestination(northwest, end)) {
				cellDetails[northwestIdx].row = point.first;
				cellDetails[northwestIdx].col = point.second;
				endFound = true;
				break;
			}
			else if (!closedList[northwestIdx] && !isBlocked(grid, northwestIdx)) {
				double g = cellDetails[index].g + 1.414;
				double h = calculateHeuristic(northwest, end);
				double f = g + h;
				
				if (cellDetails[northwestIdx].f == INF || cellDetails[northwestIdx].f > f) {
					openList.insert(std::make_pair(f, northwest));
					cellDetails[northwestIdx].f = f;
					cellDetails[northwestIdx].g = g;
					cellDetails[northwestIdx].h = h;
					cellDetails[northwestIdx].row = point.first;
					cellDetails[northwestIdx].col = point.second;
				}
			}
		}
		
		if (isValid(southeast, width, height)) {
			auto southeastIdx = southeast.first * width + southeast.second;
			if (isDestination(southeast, end)) {
				cellDetails[southeastIdx].row = point.first;
				cellDetails[southeastIdx].col = point.second;
				endFound = true;
				break;
			}
			else if (!closedList[southeastIdx] && !isBlocked(grid, southeastIdx)) {
				double g = cellDetails[index].g + 1.414;
				double h = calculateHeuristic(southeast, end);
				double f = g + h;
				
				if (cellDetails[southeastIdx].f == INF || cellDetails[southeastIdx].f > f) {
					openList.insert(std::make_pair(f, southeast));
					cellDetails[southeastIdx].f = f;
					cellDetails[southeastIdx].g = g;
					cellDetails[southeastIdx].h = h;
					cellDetails[southeastIdx].row = point.first;
					cellDetails[southeastIdx].col = point.second;
				}
			}
		}
		
		if (isValid(southwest, width, height)) {
			auto southwestIdx = southwest.first * width + southwest.second;
			if (isDestination(southwest, end)) {
				cellDetails[southwestIdx].row = point.first;
				cellDetails[southwestIdx].col = point.second;
				endFound = true;
				break;
			}
			else if (!closedList[southwestIdx] && !isBlocked(grid, southwestIdx)) {
				double g = cellDetails[index].g + 1.414;
				double h = calculateHeuristic(southwest, end);
				double f = g + h;
				
				if (cellDetails[southwestIdx].f == INF || cellDetails[southwestIdx].f > f) {
					openList.insert(std::make_pair(f, southwest));
					cellDetails[southwestIdx].f = f;
					cellDetails[southwestIdx].g = g;
					cellDetails[southwestIdx].h = h;
					cellDetails[southwestIdx].row = point.first;
					cellDetails[southwestIdx].col = point.second;
				}
			}
		} */
	}
	
	if (endFound) trace(cellDetails, end, width);
	else std::cout << "No path was found" << std::endl; 
}

#endif