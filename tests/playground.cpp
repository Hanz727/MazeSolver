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
		{ -1, -1 },
	    true 
    );
    
   	mazeSolver.markWall({ 6,6 }, 10, CompassDir::South); 
   	mazeSolver.markWall({ 6+6,6 }, 10, CompassDir::East); 
   	mazeSolver.markWall({ 6+6,6 }, 10, CompassDir::South); 
   	mazeSolver.markWall({ 8,0 }, 10, CompassDir::North); 
    mazeSolver.markWall({ 6,6 }, 10, CompassDir::West); 
    
    int steps = 1;
    double angle = 0.;
    for (int i = 0; i < steps; i++) {
	    vec2<int> nm = mazeSolver.getNextMove(angle);
        
        std::cout << "pos: (" << mazeSolver.getCurrPos().x << ", " << mazeSolver.getCurrPos().y << ") nextPos: (" << nm.x << ", " << nm.y << ") angle: " << angle << "\n";
        angle = atan2(nm.y-mazeSolver.getCurrPos().y, nm.x-mazeSolver.getCurrPos().x) + 0.5*PI_d;
        mazeSolver.setCurrPos(nm);
    }
    
    mazeSolver.printDists();
    mazeSolver.printWalls();
}
