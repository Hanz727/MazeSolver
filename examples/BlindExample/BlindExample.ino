#include "MazeSolver.h"

MazeSolver mazeSolver;

int main() {
    constexpr int w = 7;
    constexpr int h = 7;
	mazeSolver.init(
		4.,  // 4cm between cells
		20., // 20cm wide cells
		20., // 20cm high cells
		w * 2 - 1, // make the supermaze (2x-1) (13x13)
		h * 2 - 1,
		{ w-1,h-1 }, // middle of the supermaze startPos(6,6)
		{ -1,-1 }, // random value, doesn't matter
        true // blind
	);

   	mazeSolver.markWall(mazeSolver.getStartPos(), 10, CompassDir::East);
    vec2<int> nm = mazeSolver.getNextMove(0); // returns (6,5), which means one up 
    mazeSolver.setCurrPos({6,5});
    
    nm = mazeSolver.getNextMove(0); // returns (7,5), which means 1 right
    mazeSolver.setCurrPos({7,5});

    // we turn 0.5pi (right)
    nm = mazeSolver.getNextMove(0.5*PI); // returns (7,6), which means 1 down, as we go right yet again
}
