#include "MazeSolver.h"
#include "FixedDeque.h"
#include <math.h>

#ifdef DEBUG_PRINT_WINDOWS
#include <iostream>
#endif

MazeSolver::MazeSolver(
    const double wallWidth,
    const double cellWidth,
    const double cellHeight,
    const uint8_t mazeWidth,
    const uint8_t mazeHeight,
    const vec2<int> startPos,
    const vec2<int> endPos
) :
    m_wallWidth(wallWidth),
    m_cellWidth(cellWidth),
    m_cellHeight(cellHeight),
    m_MazeWidth(mazeWidth),
    m_MazeHeight(mazeHeight),
    m_startPos(startPos),
    m_endPos(endPos),
    m_MazeWidthEx(mazeWidth * 2 + 1),
    m_MazeHeightEx(mazeHeight * 2 + 1)
{
    clearDistanceMatrix();
    clearWallMatrix();
}

void MazeSolver::clearDistanceMatrix() {
    for (int x = 0; x < m_MazeWidth; x++) {
        for (int y = 0; y < m_MazeHeight; y++) {
            m_distanceMatrix[x][y] = -1;
        }
    }
}

void MazeSolver::clearWallMatrix() {
    for (int x = 0; x < m_MazeWidthEx; x++) {
        for (int y = 0; y < m_MazeHeightEx; y++) {
            bool wallState = 0;
            if (x == 0 || y == 0)
                wallState = 1;

            if (x == m_MazeWidthEx - 1 || y == m_MazeHeightEx - 1)
                wallState = 1;

            m_wallMatrix[x][y] = wallState;
        }
    }
}


vec2<double> MazeSolver::posToCm(const vec2<double>& pos) const {
    return vec2<double>(
        (m_cellWidth + m_wallWidth) * pos.x + (m_wallWidth + m_cellWidth / 2.),
        (m_cellHeight + m_wallWidth) * pos.y + (m_wallWidth + m_cellHeight / 2.)
    );
}

vec2<double> MazeSolver::cmToPos(const vec2<double>& cm) const {
    return vec2<double>(
        (cm.x - m_wallWidth - m_cellWidth / 2.) / (m_cellWidth + m_wallWidth),
        (cm.y - m_wallWidth - m_cellHeight / 2.) / (m_cellHeight + m_wallWidth)
    );
}

vec2<int> MazeSolver::roundPos(const vec2<double>& pos) const {
    return vec2<int>{(int)round(pos.x), (int)round(pos.y)};
}

vec2<int> MazeSolver::posToPosEx(const vec2<double>& pos) const {
    return vec2<int>(
        round(pos.x * 2. + 1.),
        round(pos.y * 2. + 1.)
    );
}

vec2<double> MazeSolver::posExToPos(const vec2<int>& posEx) const {
    return vec2<int>(
        (posEx.x - 1.) / 2.,
        (posEx.y - 1.) / 2.
    );
}


// Parameters:
// - pos: The current position in the maze grid, represented as a vec2<double>. The more precise the position the better.
// - distance: The distance to the wall in centimeters (double). This value is used
//   to calculate the position of the wall based on the direction.
// - dir: The direction from the current position in which the wall is located, 
//   represented by the CompassDir enum (e.g., North, South, East, West). Where north goes towards y = 0 and West goes to x = 0
void MazeSolver::markWall(const vec2 <double>& pos, const double distance, const CompassDir dir) {
    // update latest pos
    m_currPos = pos;

    vec2<double> wallPosCm = posToCm(pos) + (vec2<double>{ distance, distance }*getDirOffset(dir));
    vec2<int> wallPosEx = posToPosEx(cmToPos(wallPosCm));

    // walls are only on even spots
    if (!(wallPosEx.x % 2 == 0 || wallPosEx.y % 2 == 0))
        return;

    //std::cout << wallPosEx.x << " " << wallPosEx.y << "\n";

    m_wallMatrix[wallPosEx.x][wallPosEx.y] = 1;
}

// Parameters:
// - pos: The current position in the maze grid, represented as a vec2<double>. The more precise the position the better.
// - distance: The distance to the wall in centimeters (double). This value is used
//   to calculate the position of the wall based on the direction.
// - angleRad: The compass angle from the center of the car, 0 is North, 1/2pi is East, pi is South, 3/2pi is West. 
//             The angle is not relative to the car!
void MazeSolver::markWall(const vec2<double>& pos, const double distance, const double angleRad) {
    m_currPos = pos;

    vec2<double> wallPosCm = posToCm(pos) + (vec2<double>{ distance*sin(angleRad), distance*cos(angleRad) });
    vec2<int> wallPosEx = posToPosEx(cmToPos(wallPosCm));

    // walls are only on even spots
    if (!(wallPosEx.x % 2 == 0 || wallPosEx.y % 2 == 0))
        return;

    //std::cout << wallPosEx.x << " " << wallPosEx.y << "\n";

    m_wallMatrix[wallPosEx.x][wallPosEx.y] = 1;

}

void MazeSolver::floodFill(const vec2<int>& destination) {
    clearDistanceMatrix();
    m_distanceMatrix[destination.x][destination.y] = 0;

    FixedDeque<FloodFillNode> queue(m_MazeWidth * m_MazeHeight);
    queue.push_back({ destination, 0 });

    while (!(queue.is_empty())) {
        FloodFillNode node = queue.pop_front();

        for (vec2<int> dir : m_directions) {
            vec2<int> newPos = node.pos + dir;

            // Skip OOB
            if (newPos.x < 0 || newPos.y < 0 || newPos.x >= m_MazeWidth || newPos.y >= m_MazeHeight)
                continue;

            // Skip visited
            if (m_distanceMatrix[newPos.x][newPos.y] != -1)
                continue;

            vec2<int> newPosEx = posToPosEx(newPos);
            vec2<int> wallPos = newPosEx - dir; // step back

            // Can't drive through walls
            if (m_wallMatrix[wallPos.x][wallPos.y] == 1)
                continue;

            m_distanceMatrix[newPos.x][newPos.y] = node.dist + 1;

            queue.push_back({ newPos, node.dist + 1 });
        }

    }
}

vec2<int> MazeSolver::getDirOffset(const CompassDir dir) const {
    return m_directions[(int)(log((int)dir) / log(2))];
}

vec2<int> MazeSolver::getNextMove() const {
    static vec2<int> lastMove = roundPos(m_currPos);
    directions dirs = getPossibleMoves();

    vec2<int> bestMove{ -1 };
    int bestDist = 9999;

    for (int i = 0; i < 4; i++) {
        int8_t dir = dirs & (1 << i);
        if (!dir)
            continue;

        vec2<int> newPos = roundPos(m_currPos) + getDirOffset((CompassDir)dir);

        // Skip previous move
        if (newPos == lastMove)
            continue;

        int dist = m_distanceMatrix[newPos.x][newPos.y];
        if (dist < bestDist) {
            bestDist = dist;
            bestMove = newPos;
        }
        // TODO: add priority path switching between east, west and north, south
    }

    // Check previous move as last
    if (m_distanceMatrix[lastMove.x][lastMove.y] < bestDist)
        bestMove = lastMove;

    lastMove = bestMove;
    return bestMove;
}

directions MazeSolver::getPossibleMoves() const {
    directions out = 0;
    for (int i = 0; i < 4; i++) {
        vec2<int> wallPos = posToPosEx(m_currPos) + m_directions[i];
        if (m_wallMatrix[wallPos.x][wallPos.y] == 1)
            continue;

        out |= (1 << i);
    }

    return out;
}


void MazeSolver::printWalls() const {
#ifdef DEBUG_PRINT_WINDOWS
    for (int y = 0; y < m_MazeHeightEx; y++) {
        for (int x = 0; x < m_MazeWidthEx; x++) {
            std::cout << (int)(m_wallMatrix[x][y]) << " ";
        }
        std::cout << "\n";
    }
#endif
}

void MazeSolver::printDists() const {
#ifdef DEBUG_PRINT_WINDOWS
    for (int y = 0; y < m_MazeHeight; y++) {
        for (int x = 0; x < m_MazeWidth; x++) {
            std::cout << (int)(m_distanceMatrix[x][y]) << " ";
        }
        std::cout << "\n";
    }
#endif
}
