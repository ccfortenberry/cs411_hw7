/**************************************
*	Curtis Fortenberry
*	CS411
*	HW7
*	Chris Hartman
**************************************/
/**************************************
*	astar.cpp
*	C++ A* Search implementation
**************************************/
#include "astar.hpp"
using astar::grid_type;
using astar::pair_type;
using astar::aStar;

//vector included
using std::vector;
//iostream included
using std::cout;
using std::endl;
#include <algorithm>
using std::generate;
#include <random>
std::random_device rd;
std::mt19937 gen(rd());
std::uniform_int_distribution<> d(0, 1);

int main() {
	{
		cout << "Test grid #1" << endl;
		grid_type grid = { 
			1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 
			1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 
			1, 1, 1, 0, 1, 1, 0, 1, 0, 1, 
			0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 
			1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 
			1, 0, 1, 1, 1, 1, 0, 1, 0, 0, 
			1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 
			1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 
			1, 1, 1, 0, 0, 0, 1, 0, 0, 1 
		};
		
		pair_type start = {8,0};
		pair_type end = {0,0};
		const int COLS = 10;
		const int ROWS = 9;
		
		for (unsigned int i=0; i < COLS*ROWS; i++) {
			if (i%COLS == 0) cout << "\n" << grid[i] ;
			else cout << grid[i];
		}
		
		cout << endl << "Start: (" << start.first << "," << start.second << ")\n";
		cout << "End: (" << end.first << "," << end.second << ")\n";
		
		aStar(grid, start, end, COLS, ROWS);
		cout << endl;
	}
	{
		cout << "Test grid #2" << endl;
		grid_type grid = {
			1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 
			1, 0, 1, 0, 1, 1, 1, 0, 1, 1, 
			1, 1, 1, 0, 1, 1, 0, 1, 0, 1, 
			1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 
			1, 0, 0, 0, 1, 1, 1, 0, 1, 0, 
			1, 0, 1, 1, 1, 1, 0, 1, 0, 0, 
			1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 
			1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 
			1, 1, 1, 0, 0, 0, 1, 0, 0, 1 
		};
	
		pair_type start = {8,0};
		pair_type end = {0,0};
		const int COLS = 10;
		const int ROWS = 9;
		
		for (unsigned int i=0; i < COLS*ROWS; i++) {
			if (i%COLS == 0) cout << "\n" << grid[i] ;
			else cout << grid[i];
		}
		
		cout << endl << "Start: (" << start.first << "," << start.second << ")\n";
		cout << "End: (" << end.first << "," << end.second << ")\n";
		
		aStar(grid, start, end, COLS, ROWS);
		cout << endl;
	}
	{
		cout << "Test grid #3" << endl;
		grid_type grid = {
			1, 0, 1, 1, 0, 1, 0, 1, 1, 1, 
			0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 
			0, 1, 1, 0, 1, 1, 0, 1, 0, 1, 
			0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 
			0, 0, 0, 0, 1, 1, 1, 0, 1, 0, 
			0, 0, 1, 1, 1, 1, 0, 1, 0, 1, 
			1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 
			1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 
			1, 0, 1, 0, 0, 1, 0, 1, 0, 1, 
			1, 1, 1, 0, 0, 0, 1, 0, 0, 1 
		};
	
		pair_type start = {6,0};
		pair_type end = {0,0};
		const int COLS = 10;
		const int ROWS = 10;
		
		for (unsigned int i=0; i < COLS*ROWS; i++) {
			if (i%COLS == 0) cout << "\n" << grid[i] ;
			else cout << grid[i];
		}
		
		cout << endl << "Start: (" << start.first << "," << start.second << ")\n";
		cout << "End: (" << end.first << "," << end.second << ")\n";
		
		aStar(grid, start, end, COLS, ROWS);
		cout << endl;
	}
	{
		cout << "Test grid #4: The Random Test (12x5)" << endl;
		grid_type grid(12*5);
		generate(grid.begin(), grid.end(), [](){ return d(gen); });
	
		pair_type start = {0,0};
		pair_type end = {11,4};
		const int COLS = 12;
		const int ROWS = 5;
		
		for (unsigned int i=0; i < COLS*ROWS; i++) {
			if (i%COLS == 0) cout << "\n" << grid[i] ;
			else cout << grid[i];
		}
		
		cout << endl << "Start: (" << start.first << "," << start.second << ")\n";
		cout << "End: (" << end.first << "," << end.second << ")\n";
		
		aStar(grid, start, end, COLS, ROWS);
		cout << endl;
	}
	return 0;
}