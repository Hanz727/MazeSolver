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
		{ 1,0 }
	);

   	//mazeSolver.markWall({ 0,0 }, 10, CompassDir::East);
	mazeSolver.floodFill(mazeSolver.m_endPos);
	vec2<int> nm = mazeSolver.getNextMove();
    
    std::cout << nm.x << " " << nm.y << "\n";

    mazeSolver.printWalls();
}
