#pragma once
#include "vec2.h"
#include "FixedDeque.h"
#include "pch.h"

#define PI_d 3.141592653589793
#define INVALID_ANGLE -123.45 

// Max maze size is fixed on 10 by 10
using matrix2d = int8_t[10*2-1][10*2-1];
using matrix2dEx = int8_t[10*4-1][10*4-1];

using directionFlags = uint8_t;

enum CompassDir : uint8_t {
    North = (1 << 0),
    South = (1 << 1),
    East = (1 << 2),
    West = (1 << 3)
};

enum class Stage : uint8_t {
    BOUND_SEARCH, // NOTHING IS KNOWN YET
    EXIT_SEARCH  // WE KNOW THE BOUNDS
};

struct FloodFillNode {
    vec2<int> pos;
    int dist;
};

class MazeSolver {
private:
    double m_wallWidth;
    double m_cellWidth;
    double m_cellHeight;

    uint8_t m_mazeWidth;
    uint8_t m_mazeHeight;
    uint8_t m_mazeWidthEx;
    uint8_t m_mazeHeightEx;

    matrix2d m_distanceMatrix{};
    matrix2d m_visitedMatrix{};
    matrix2dEx m_wallMatrix{};

    const vec2<int8_t> m_directions[4] = {
        { 0,-1}, // N
        { 0, 1}, // S
        { 1, 0}, // E
        {-1, 0}  // W
    };

    // Priority, Lower number = higher priority
    uint8_t m_movePriority[4] = {
        1, // N
        3, // S
        0, // E
        2, // W
    };

    vec2<int> m_startPos;
    vec2<int> m_endPos;
    vec2<double> m_currPos;
    
    bool m_explorationMode = true;

    bool m_blind = false;
    Stage m_blindStage = Stage::BOUND_SEARCH;
    vec2<int> m_topLeft{999,999};     // TOP LEFT corner of maze, found by bound search
    vec2<int> m_bottomRight{-1,-1};
    bool m_atExit = false;

    void clearDistanceMatrix();
    void clearWallMatrix();
    
    void floodFill(FixedDeque<FloodFillNode>& queue);
    void floodFillBorders();
    void floodFillUnvisited();

    bool findBounds();
    uint8_t dirToIndex(CompassDir dir) const;
private:
    vec2<double> posToCm(const vec2<double>& pos) const;
    vec2<double> cmToPos(const vec2<double>& cm) const;
    vec2<int> roundPos(const vec2<double>& pos) const;

    CompassDir radiansToDirection(double angleRad) const;
    double directionToRadians(CompassDir dir) const;

    // This is int because this function is only used to get the wall position which must be int.
    // So it rounds to the nearest wall
    vec2<int> posToPosEx(const vec2<double>& pos) const;
    vec2<double> posExToPos(const vec2<int>& posEx) const;

    uint8_t* getMovesOrder(directionFlags directions, uint8_t* size, double offsetRad = 0.) const;
public:
    MazeSolver(const double wallWidth,
        const double cellWidth,
        const double cellHeight,
        const uint8_t mazeWidth,
        const uint8_t mazeHeight,
        const vec2<int> startPos,
        const vec2<int> endPos,
        const bool blind = false
    );

    MazeSolver() = default;
    
    ~MazeSolver() = default;
   
    void init(const double wallWidth,
        const double cellWidth,
        const double cellHeight,
        const uint8_t mazeWidth,
        const uint8_t mazeHeight,
        const vec2<int> startPos,
        const vec2<int> endPos,
        const bool blind = false
    );

    void setMovePriority(directionFlags priority[4]);
    void setExplorationMode(bool toggle);
    void setCurrPos(const vec2<double>& pos);
    
    const vec2<int>& getStartPos() const; 
    const vec2<double>& getCurrPos() const; 
    const vec2<int>& getEndPos() const; 
    bool atExit();

    void markWall(const vec2<double>& pos, double distance, CompassDir dir);
    void markWall(const vec2<double>& pos, double distance, double angleRad);
    void floodFill(const vec2<int>& destination);

    vec2<int> getDirOffset(CompassDir dir) const;
    vec2<int> getNextMove(double carBearing = 0.);
    directionFlags getPossibleMoves() const;

    vec2<int> projectPos(const vec2<double>& pos, double distance, double angleRad) const;

    void printWalls() const;
    void printDists() const;
};

