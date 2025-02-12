# MazeSolver
This module exists to map and solve the maze, aswell as navigate around it to any desired position.

First init the MazeSolver with:
```c
MazeSolver mazeSolver(
        const double wallWidth,
        const double cellWidth,
        const double cellHeight,
        const uint8_t mazeWidth,
        const uint8_t mazeHeight,
        const vec2<int> startPos,
        const vec2<int> endPos
);
```
Alternatively:
```c
MazeSolver mazeSolver;

mazeSolver.init(        
        const double wallWidth,
        const double cellWidth,
        const double cellHeight,
        const uint8_t mazeWidth,
        const uint8_t mazeHeight,
        const vec2<int> startPos,
        const vec2<int> endPos
)
```
This can be used if you don't know the exact dimensions or start/end position yet, but can aquire that information while in the maze.


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

## Important functions


### markWall
markWall - mark a detected wall on the ```m_wallMatrix```.

### C Specification

```c
void markWall(const vec2<double>& pos,
              const double distance,
              const CompassDir dir);

void markWall(const vec2<double>& pos,
              const double distance,
              const double angleRad);
```


### Parameters

***pos*** <br> 
    &nbsp;&nbsp;&nbsp;&nbsp;Position from which the distance to the wall is measured.<br>
***distance*** <br> 
    &nbsp;&nbsp;&nbsp;&nbsp;Distance between the pos and a wall in direction dir/angleRad in centimeters. <br>
***dir*** <br> 
    &nbsp;&nbsp;&nbsp;&nbsp;Compass direction from [Enum CompassDir](#Enum-CompassDir) <br>
***angleRad*** <br> 
    &nbsp;&nbsp;&nbsp;&nbsp;Absolute angle in radians from the sensor position to the wall.
    This angle is not relative to the car, but to the maze where North is 0 and South is PI<br>

### Description

`markWall` stores the existance of a wall for later calculations with [floodFill](#floodFill).
This function has to be called at least once when roughly in the center of a cell for each sensor.
It can be called from all possible positions of a cell given the direction/angleRad is correct.
Do not use this function while turning if you aren't sure whether the angleRad is correct at that very moment. <br>

The tolerance of distance is approximately ±1/4(cellWidth-wallWidth), this tolerance is higher in the positive direction by wallWidth,
but keep the measurements within the minimum tolerance value.

___
### floodFill 
floodFill - recalculate the ```m_distanceMatrix``` with the floodfill algorithm. 

### C Specification
```c
void floodFill(const vec2<int>& destination);
```

### Parameters

***destination*** <br> 
    &nbsp;&nbsp;&nbsp;&nbsp;Where you want to arrive, can be the ```getEndPos()```, but also any other point you want to navigate to<br>

### Description
This function populates the ```m_distanceMatrix``` where 0 is the destination, -1 is unreachable and any other
integer is the number of steps away from the destination. This algorithm factors in the ```m_wallMatrix``` and thus will make a distance larger
if there is a wall blocking the direct path. [getNextMove](#getNextMove) function will follow the cell with the lowest distance to reach the destination. <br> 

Whenever using this function you first [markWall](#markWall) to ensure you have all the needed data before running the algorithm.
This function is usually followed by [getNextMove](#getNextMove) to navigate towards destination.

### getNextMove() 
(WIP)




## Custom types

### Enum CompassDir
CompassDir - A compass direction North, South, East or West.

### C Specification

```c
enum CompassDir : uint8_t {
    North = (1 << 0),
    South = (1 << 1),
    East  = (1 << 2),
    West  = (1 << 3)
};
```

### Description
(WIP)

___

Example code:
```c
// Assuming 3 sensors
mazeSolver.markWall({ 0,0 }, 10, CompassDir::East);
mazeSolver.markWall({ 0,0 }, 10, CompassDir::West);
mazeSolver.markWall({ 0,0 }, 32, CompassDir::South);

mazeSolver.floodFill(mazeSolver.getEndPos());
vec2<int> nextMove = mazeSolver.getNextMove();

// now use mazeSolver.m_currPos and nextMove to decide on the next course of action.
```

### Remarks

- It is very important that at the time of calling ```markWall()``` function, the car is parallel to maze walls. 
If the car is at an angle, don't call the function and wait until it's fully rotated.
- The distance provided to ```markWall()``` function can deviate by 
±(1/4)*(cellWidth-wallWidth).
- The distance provided to ```markWall()``` has to be from center of the car, 
not from the sensor.
- The (0,0) pos is top left of the maze
- North is y = 0
- West is x = 0
- There also is a ```markWall()``` function that supports an angle instead of CompassDir, 
it can be used for angled sensors or detecting walls while turning.
- Once a maze has been solved, ```getNextMove() == getEndPos()```, use ```floodFill(mazeSolver.getStartPos())```
to return back to the start point.
- After solving the maze and returning to the start point, use ```setExplorationMode(false)```
to only follow visited paths.

