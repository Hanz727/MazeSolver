#include "../src/MazeSolver.h"
#include <iostream>

MazeSolver mazeSolver;

int main() {
    constexpr int w = 7;
    constexpr int h = 7;
	mazeSolver.init(
		4.,
		20.,
		20.,
		w * 2 - 1,
		h * 2 - 1,
		{ w-1,h-1 },
		{ -1,-1 },
        true // blind
	);

   	mazeSolver.markWall(mazeSolver.getStartPos(), 10, CompassDir::East);
	//mazeSolver.floodFill({0,0});
    mazeSolver.setCurrPos({6,5});
    mazeSolver.setCurrPos({7,5});
    vec2<int> nm = mazeSolver.getNextMove(0.5*PI_d);
    
    mazeSolver.printDists();
    mazeSolver.printWalls();
    std::cout << nm.x << " " << nm.y << "\n";
}
