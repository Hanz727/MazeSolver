#pragma once
#include "vec2.h"

#if defined(__has_include) && __has_include(<stdint.h>)
    #include <stdint.h>
#endif

// Max maze size is fixed on 32 by 32
using matrix2d = int8_t[32][32];
using matrix2dEx = int8_t[65][65];
using moves = uint8_t;

enum CompassDir : uint8_t {
    North = (1 << 0),
    South = (1 << 1),
    East = (1 << 2),
    West = (1 << 3)
};

struct FloodFillNode {
    vec2<int> pos;
    int dist;
};

class MazeSolver {
private:
    const double m_wallWidth;
    const double m_cellWidth;
    const double m_cellHeight;

    const uint8_t m_MazeWidth;
    const uint8_t m_MazeHeight;
    const uint8_t m_MazeWidthEx;
    const uint8_t m_MazeHeightEx;

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
    moves m_movePriority[4] = {
        1, // N
        3, // S
        2, // E
        0, // W
    };

    const vec2<int> m_startPos;
    const vec2<int> m_endPos;
    vec2<double> m_currPos;
    
    bool m_explorationMode = true;

    void clearDistanceMatrix();
    void clearWallMatrix();

    uint8_t* getMovesOrder(moves _moves, uint8_t* size) const;
    
public:
    
    MazeSolver(const double wallWidth,
        const double cellWidth,
        const double cellHeight,
        const uint8_t mazeWidth,
        const uint8_t mazeHeight,
        const vec2<int> startPos,
        const vec2<int> endPos
    );

    ~MazeSolver() = default;
    
    void setMovePriority(const moves priority[4]);
    void setExplorationMode(bool toggle);
    void setCurrPos(const vec2<double>& pos);
    
    const vec2<int>& getStartPos() const; 
    const vec2<double>& getCurrPos() const; 
    const vec2<int>& getEndPos() const; 

    vec2<double> posToCm(const vec2<double>& pos) const;
    vec2<double> cmToPos(const vec2<double>& cm) const;
    vec2<int> roundPos(const vec2<double>& pos) const;

    vec2<int> posToPosEx(const vec2<double>& pos) const;
    vec2<double> posExToPos(const vec2<int>& posEx) const;

    void markWall(const vec2<double>& pos, const double distance, const CompassDir dir);
    void markWall(const vec2<double>& pos, const double distance, const double angle);
    void floodFill(const vec2<int>& destination);

    vec2<int> getDirOffset(const CompassDir dir) const;
    vec2<int> getNextMove() const;
    moves getPossibleMoves() const;

    void printWalls() const;
    void printDists() const;
};
