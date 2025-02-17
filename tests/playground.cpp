#include "../src/MazeSolver.h"
#include <iostream>

MazeSolver mazeSolver;

int main() {
	mazeSolver.init(
		1.,
		20.,
		20.,
		10,
		10,
		{ 0,0 },
		{ 9,9 }
	);
    
    moves_t m = North | South | East | West;
    double offset = 1.5*PI_d;

    uint8_t orderSize;
    uint8_t* order = mazeSolver.getMovesOrder(m, &orderSize, offset); 

    for (int i = 0; i < orderSize; i++) {
        moves_t dir = m & (1 << order[i]);
        std::cout << (int)dir << "\n";
    }

   	//mazeSolver.markWall({ 0,0 }, 10, CompassDir::East);
    //mazeSolver.floodFill({1,0});
	//vec2<int> nm = mazeSolver.getNextMove();
    
    //mazeSolver.printDists();
    //mazeSolver.printWalls();
    //std::cout << nm.x << " " << nm.y << "\n";
}
