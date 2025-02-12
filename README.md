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
It can be called from all possible positions of a cell.
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

___

### getNextMove 
getNextMove - Gives the adjecent cell of current position with lowest
distance to destination complying with ```m_movePriority```.

### C Specification

```c
vec2<int> getNextMove();
```

### Description

```getNextMove``` returns a adjecent cell to ```m_currPos``` with
the lowest distance to destination provided to [floodFill](#floodFill).
If there are multiple adjecent cells with equal distance to the 
destination, then ```m_movePriority``` is followed. The previous
position always has the lowest priority to avoid going back and fourth. <br>

The adjescent cell is returned as a grid vec2<int>
where top left cell is (0,0)

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

A listing of directions where North faces y = 0 and West faces x = 0.
This enum is used by ```getPossibleMoves``` to return multiple
possible moves without creating a vec2 array. For example:
```c
moves_t possibleMoves = North | South;
```
This becomes 0b0011 and represents that you can go North and South,
but not East or West.

You can translate a single compassDir to its offset 
with ```getDirOffset```.

For example:
```c
getDirOffset((1 << 0)); // returns vec2<int>{0, -1}
getDirOffset(South);    // returns vec2<int>{0, 1}
```

___

 After solving the maze and returning to the start point, use ```setExplorationMode(false)```
to only follow visited paths.

