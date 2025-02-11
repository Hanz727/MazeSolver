#include "../src/MazeSolver.h"
#include <iostream>

int main() {
	MazeSolver mazeSolver(
		1.,
		20.,
		20.,
		10,
		10,
		{ 0,0 },
		{ 9,9 }
	);

   	mazeSolver.markWall({ 0,0 }, 10, CompassDir::East);
	mazeSolver.floodFill({1,0});
	vec2<int> nm = mazeSolver.getNextMove();
    
    mazeSolver.printDists();
    mazeSolver.printWalls();
    std::cout << nm.x << " " << nm.y << "\n";
}
