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
        const vec2<int> endPos,
        const bool blind
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
        const vec2<int> endPos,
        const bool blind
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
    {9,9},
    false
);
```
or for blind mode:
```c
MazeSolver mazeSolver(
    1.,
    20.,
    20.,
    10*2-1, // width * 2 - 1
    10*2-1, // height * 2 - 1
    {10-1,10-1}, // width - 1, height - 1
    {-1,-1}, // doesn't matter what is here, there is no end point in blind mode
    true // blind mode on
);
```

## Grid system
Firstly there are cell positions, usually not distinguished by any pre/sub fix, 
often just called `pos`. (0,0) pos is top-left of the grid. Any cell can have up to four walls around it, 
blocking the entrance to the adjacent cell. Position doesn't need to be an integer and (0.5,0) means we are exactly
half way between two cells, inside a wall. To correlate this arbitrary position there is `cm` or `posCm`.
This "position" is the offset in centimeters from the top-left corner (from outside the wall) to cell position.
Meaning that if a wall is 1 cm thick and cell is 10cm by 10cm, pos (0,0) is equal to posCm(11,11).
To store the walls there is another coordinate system often called posEx, meaning extended.
`m_mazeWidthEx` and it's height equivalent are 2x+1 bigger than the normal grid (`m_mazeWidth`).
posEx(1,1) is equal to pos(0,0), posEx(0,1) is the wall left to pos(0,0) and posEx(2,1) is the wall right of pos(0,0).
Within this grid we can see "corner walls", such as posEx(0,0), but this is disregarded and used as a buffer between walls to prevent 
accidentally detecting the wrong wall. <br>

In blind mode, there also is a `supermaze`. Supermaze contains all possible translations of a smaller maze where supermaze's middle point is contained
within the smaller maze. This ensures that the start position is fixed, but you can still be at any position in the smaller maze. It's size is
2x-1 the smaller maze and its middle point and start point is always x-1 where x is width or height.

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
This function is usually followed by [getNextMove](#getNextMove) to navigate towards destination. <br>

Do NOT use this function in blind mode. A different type of floodFill is called from within the [getNextMove](#getNextMove) function.
___

### getNextMove 
getNextMove - Gives the adjecent cell to current position that has 
the lowest distance to destination or uses the DimSearch algorithm in blind mode. 

### C Specification

```cpp
vec2<int> getNextMove(const double carBearing = 0.f);
```


### Parameters

***carBearing*** <br> 
    &nbsp;&nbsp;&nbsp;&nbsp;Car bearing in radians where North of the maze is 0 and south is pi. 
                            Ignore this if not in blind mode<br>

### Description

In normal mode, ```getNextMove``` returns a adjecent cell to ```m_currPos``` with
the lowest distance to destination provided to [floodFill](#floodFill).
If there are multiple adjecent cells with equal distance to the 
destination, then ```m_movePriority``` is followed. The previous
position always has the lowest priority to avoid going back and fourth. <br>

In blind mode, it will first attempt to identify the bounds of the maze and then switch to
floodFill with destination on each outer cell where no wall was yet detected. <br>

The adjescent cell is returned as a grid vec2<int>
where top left cell is (0,0)

___
### setExplorationMode 
setExplorationMode - change the current explorationMode, true by default. 

### C Specification
```c
void setExplorationMode(bool toggle);
```

### Parameters

***toggle*** <br> 
    &nbsp;&nbsp;&nbsp;&nbsp;New value of ```m_explorationMode``` <br>


### Description
The exploration mode dictates whether the [getNextMove](#getNextMove)
function will ignore non visited cells.
When ```m_explorationMode``` is true, [getNextMove](#getNextMove)
can return any cell, ignoring ```m_visitedMatrix```. <br>

The main use case for this function is to turn off the exploration mode
when you want to set the best time with current maze knowledge 
without taking any risks.

___
### projectPos 
projectPos - projects polar coordinates on the grid and returns the closest position.

### C Specification

```cpp
vec2<int> projectPos(const vec2<double>& pos, 
                     const double distance, 
                     const double angleRad);
```

### Parameters

***pos*** <br> 
    &nbsp;&nbsp;&nbsp;&nbsp;Position from which the distance is measured.<br>
***distance*** <br> 
    &nbsp;&nbsp;&nbsp;&nbsp;Distance in centimeters. <br>
***angleRad*** <br> 
    &nbsp;&nbsp;&nbsp;&nbsp;Absolute angle in radians from the sensor position.
    This angle is not relative to the car, but to the maze where North is 0 and South is PI<br>

### Description

```projectPos``` can be used to calculate the position of another car
or a signal in the maze using polar coordinates.
If you know that something is 30 degrees right of the car, 
the orientation of the car and the distance to it, you can calculate
its position.


## Custom structures

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


## Blind mode
This mode, stored as ```m_blind``` is meant to solve a maze only with it's dimensions,
without the knowledge of startPos or endPos. In order to use it you must set the
maze width and height as 2*x-1. This is beacuse we are making enough place for the middle
of the maze to be a corner of the actual maze. This larger maze is called the supermaze.
In this mode there are no walls placed around the edges of the maze as there may be an exit there. <br>

The startPos has to be set to the center of the supermaze.
Easy way to do this is to set it to (width-1, height-1), where width and height are the dimensions of the maze (not supermaze) <br>

In blind mode, there are two stages. ```Stage::BOUND_SEARCH``` and ```Stage::FOLLOW_SIDES```
In the bound search stage we are looking for 2 vertical walls separated by maze width and 2 horizontal walls separated by maze height.
After that the stage is switched as we know where to place the zeroes in floodFill.
In the second stage we simply go to the closest outer wall which can potentially be an exit.
