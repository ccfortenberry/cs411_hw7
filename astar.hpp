/**************************************
*	Curtis Fortenberry
*	CS411
*	HW7
*	Chris Hartman
**************************************/
/**************************************
*	astar.hpp
*	C++ A* Search implementation
**************************************/
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

/************************************************
*	A* Search
*	Goal:
*	- Find an approximation of the shortest path 
*	from a given start and end point.
*	- Nodes are picked based on a value 'f'
*	-- f: g+h
*	--- g: movement cost from a cell to an adjacent
*		cell on the grid
*	--- h: heuristic | estimated cost of moving
*		from this cell to the end cell
************************************************/
namespace astar {
	using pair_type = std::pair<int,int>;
	using dpair_type = std::pair<double, pair_type>;
	using grid_type = std::vector<int>;

	struct Cell {
		int row, col;
		double f, g, h;
	};

	bool isInBounds(const pair_type & point, const int & width, const int & height) {
		return (point.first >= 0) && (point.first < width) && (point.second >= 0) && (point.second < height);
	}

	bool isBlocked(const grid_type & grid, int index) {
		return grid[index] == 0;
	}

	bool isDestination(const pair_type & point, const pair_type & dest) {
		return point == dest;
	}

	double calculateHeuristic(const pair_type & point, const pair_type & dest) {
		return std::sqrt((point.first - dest.first) * (point.first - dest.first) + (point.second - dest.second) * (point.second - dest.second));
	}

	bool checkAdjCell(const grid_type & grid, const pair_type & point, const pair_type & adj, const int & index, const int & width, const int & height, const pair_type & dest, const std::vector<bool> & closedList, std::set<dpair_type> & openList, std::vector<Cell> & cellDetails) {
		auto tempIdx = adj.first * width + adj.second;
		
		if (isInBounds(adj, width, height)) {
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

	void aStar(const grid_type & grid, const pair_type & start, const pair_type & end, const int & width, const int & height) {
		// Initial bounds checking to ensure that 
		// the grid and points passed are in a usable state.
		if (!isInBounds(start, width, height)) { std::cout << "ERROR: Invalid Start point (Out Of Bounds)!" << std::endl; return; }
		if (!isInBounds(end, width, height)) { std::cout << "ERROR: Invalid End point (Out Of Bounds)!" << std::endl; return; }
		
		auto startIdx = start.first * width + start.second;
		auto endIdx = end.first * width + end.second;
		
		if (isBlocked(grid, startIdx) || isBlocked(grid, endIdx)) { std::cout << "ERROR: Blocked Start/End point!" << std::endl; return; }
		if (isDestination(start, end)) { std::cout << "ERROR: Start/End point are the same point!" << std::endl; return; }
		
		// Initialize the data structures 
		std::vector<bool> closedList(width*height, false);
		std::set<dpair_type> openList;
		openList.insert(std::make_pair(0.0, start));
		
		std::vector<Cell> cellDetails(width*height, {-1, -1, INF, INF, INF}); // <- It's a surprise tool that will help us later
		
		cellDetails[startIdx].f = 0.0;
		cellDetails[startIdx].g = 0.0;
		cellDetails[startIdx].h = 0.0;
		cellDetails[startIdx].row = start.first;
		cellDetails[startIdx].col = start.second; // It's for Dynamic Programming
		
		bool endFound = false;
		
		// Run the Algorithm!
		do {
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
			
		} while (!openList.empty() && !endFound);
		
		if (endFound) trace(cellDetails, end, width);
		else std::cout << "No path was found..." << std::endl; 
	}
}

#endif