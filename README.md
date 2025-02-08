# MazeSolver usage guide
This module exists to map and solve the maze.

First init the MazeSolver with:
```c
MazeSolver(const double wallWidth,
    const double cellWidth,
    const double cellHeight,
    const uint8_t mazeWidth,
    const uint8_t mazeHeight,
    const vec2<int> startPos,
    const vec2<int> endPos
);
```

Example code:
```c
MazeSolver mazeSolver(
    1.,
    20.,
    20.,
    10,
    10,
    {0,0},
    {9,9}
);
```

Each update() iteration must do the following in order:
- markWall (for each sensor) -> updates wall information and current position
- floodFill -> recalculates the distances to endPos with the potentially new wall information

Now you can use ```getNextMove()``` to decide what the car should do.

Example code:
```c
// Assuming 3 sensors
mazeSolver.markWall({ 0,0 }, 10, CompassDir::East);
mazeSolver.markWall({ 0,0 }, 10, CompassDir::West);
mazeSolver.markWall({ 0,0 }, 32, CompassDir::South);

mazeSolver.floodFill(mazeSolver.m_endPos);
vec2<int> nextMove = mazeSolver.getNextMove();

// now use mazeSolver.m_currPos and nextMove to decide on the next course of action.
```

### Remarks

- It is very important that at the time of calling ```markWall()``` function, the car is parallel to maze walls. 
If the car is at an angle, don't call the function and wait until it's fully rotated.
- The distance provided to ```markWall()``` function can deviate by ±(1/4)*(cellWidth-wallWidth).
- The distance provided to ```markWall()``` has to be from center of the car, not from the sensor.
- The (0,0) pos is top left of the maze
- North is y = 0
- West is x = 0
- There also is a ```markWall()``` function that supports an angle instead of CompassDir, it can be used for angled sensors or
detecting walls while turning.
