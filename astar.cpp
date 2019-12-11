#include "astar.hpp"

//vector included
using std::vector;

int main() {
	{
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
		
		aStar(grid, start, end, COLS, ROWS);
	}
	{
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
		
		aStar(grid, start, end, COLS, ROWS);
	}
	return 0;
}