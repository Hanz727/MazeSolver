#include "../src/MazeSolver.h"
#include <iostream>

MazeSolver mazeSolver;

int main() {
	mazeSolver.init(
		4.,
		20.,
		20.,
		7 * 2 - 1,
		7 * 2 - 1,
		{ 7-1,7-1 },
		{ 0, 0 },
	    true
    );

        
    //uint8_t orderCount;
    //uint8_t* order = mazeSolver.getMovesOrder(15, &orderCount, 0.5*PI_d);
    //for (int i = 0; i < orderCount; i++) {
    //    std::cout << (1 << order[i]) << "\n";
    //}
    //delete[] order;

    mazeSolver.markWall(mazeSolver.getCurrPos(), 10, CompassDir::East); 
    mazeSolver.markWall(mazeSolver.getCurrPos(), 10, CompassDir::West); 

    int steps = 13;
    double angle = 0.;
    for (int i = 0; i < steps; i++) {
	    vec2<int> nm = mazeSolver.getNextMove(angle);

        //std::cout << "pos: (" << mazeSolver.getCurrPos().x << ", " << mazeSolver.getCurrPos().y << ") nextPos: (" << nm.x << ", " << nm.y << ") angle: " << angle << " at exit: " << mazeSolver.atExit() << "\n";
        angle = atan2(nm.y-mazeSolver.getCurrPos().y, nm.x-mazeSolver.getCurrPos().x) + 0.5*PI_d;
        mazeSolver.setCurrPos(nm);
        if (i < 5) {
            mazeSolver.markWall(mazeSolver.getCurrPos(), 10, CompassDir::East); 
            mazeSolver.markWall(mazeSolver.getCurrPos(), 10, CompassDir::West); 
        }

        if (i == 5) {
            mazeSolver.markWall(mazeSolver.getCurrPos(), 10, CompassDir::East); 
            mazeSolver.markWall(mazeSolver.getCurrPos(), 10, CompassDir::North); 
        }

        if (i > 5 && i != 11) {
            mazeSolver.markWall(mazeSolver.getCurrPos(), 10, CompassDir::North); 
            mazeSolver.markWall(mazeSolver.getCurrPos(), 10, CompassDir::South); 
        }
        
        if (i == 11) {
            mazeSolver.markWall(mazeSolver.getCurrPos(), 10, CompassDir::West); 
            mazeSolver.markWall(mazeSolver.getCurrPos(), 10, CompassDir::North); 
        }

    }
    
    mazeSolver.printDists();
    mazeSolver.printWalls();
}
