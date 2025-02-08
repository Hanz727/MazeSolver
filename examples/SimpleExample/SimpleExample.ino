#include <Arduino.h>
#include <MazeSolver.h>

MazeSolver gMazeSolver(
        1., // 1 cm wall width
        20., // 20 cm cell width
        20., // 20 cm cell height/breadth
        10,  // 10 cells width
        10,  // 10 cells height
        { 0,0 }, // top left start position
        { 9,9 }  // bottom right end position
);

void setup() {
    
}

void update() {
    // current pos is 0,0. Top Left
    // distance is 10 (half cell)
    // The direction is East, so we have a wall between (0,0) and (1,0)
    gMazeSolver.markWall({0,0}, 10, CompassDir::East);
    
    // current pos is still 0,0. Top Left
    // distance is 10 (half cell)
    // The direction is West, equivalent to 1.5PI (3/4 circle). So we see a wall 10cm left/west of (0,0)
    gMazeSolver.markWall({0,0}, 10, 1.5*PI);
    
    // Recompute a distance matrix for floodfill with the new wall data.
    // destination is endpos
    gMazeSolver.floodFill(gMazeSolver.m_endPos);
    
    // In this case we have a wall left and right so only next position can be (0,1)
    // Meaning 1 down.
    vec2<int> nextMove = gMazeSolver.getNextMove();
}
