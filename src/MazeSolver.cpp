#include "MazeSolver.h"
#include <string.h>
#include <math.h>

#ifdef DEBUG
#include <iostream>
#include <iomanip>
#endif
#ifndef min
#define min(a, b) ((a) < (b) ? (a) : (b))
#endif

#ifndef max
#define max(a, b) ((a) > (b) ? (a) : (b))
#endif

MazeSolver::MazeSolver(
    const double wallWidth,
    const double cellWidth,
    const double cellHeight,
    const uint8_t mazeWidth,
    const uint8_t mazeHeight,
    const vec2<int> startPos,
    const vec2<int> endPos,
    const bool blind 
)
{
    m_currPos = m_startPos;

    init(
        wallWidth,
        cellWidth,
        cellHeight,
        mazeWidth,
        mazeHeight,
        startPos,
        endPos,
        blind
    );

}

void MazeSolver::init(
        const double wallWidth,
        const double cellWidth,
        const double cellHeight,
        const uint8_t mazeWidth,
        const uint8_t mazeHeight,
        const vec2<int> startPos,
        const vec2<int> endPos,
        const bool blind
) {

    m_wallWidth = wallWidth;
    m_cellWidth = cellWidth;
    m_cellHeight = cellHeight;
    m_MazeWidth = mazeWidth;
    m_MazeHeight = mazeHeight;
    m_startPos = startPos;
    m_endPos = endPos;
    m_blind = blind;
    
    m_MazeHeightEx = mazeHeight * 2 + 1;
    m_MazeWidthEx = mazeHeight * 2 + 1;

    clearDistanceMatrix();
    clearWallMatrix();
}

void MazeSolver::clearDistanceMatrix() {
    // Warning memset only works properly for 0 and -1.
    memset(m_distanceMatrix, -1, sizeof(m_distanceMatrix));
}

void MazeSolver::clearWallMatrix() {
    for (int x = 0; x < m_MazeWidthEx; x++) {
        for (int y = 0; y < m_MazeHeightEx; y++) {
            bool wallState = 0;
            
            // initialize all walls to zero in blind mode
            if (m_blind) {
                m_wallMatrix[x][y] = wallState;
                continue; 
            }

            if (x == 0 || y == 0)
                wallState = 1;

            if (x == m_MazeWidthEx - 1 || y == m_MazeHeightEx - 1)
                wallState = 1;

            m_wallMatrix[x][y] = wallState;
        }
    }
}


vec2<int> MazeSolver::projectPos(const vec2<double>& pos, const double distance, const double angleRad) const {
    return roundPos(cmToPos(posToCm(pos) + vec2<double>{distance*sin(angleRad), distance*cos(angleRad)}));
}

void MazeSolver::setMovePriority(const moves_t priority[4]) {
    for (int i = 0; i < 4; i++) {
        m_movePriority[i] = priority[i];
    }
}

uint8_t MazeSolver::dirToIndex(CompassDir dir) const {
    if (dir == North)
        return 0;
    if (dir == South)
        return 1;
    if (dir == East)
        return 2;
    return 3;
}

uint8_t* MazeSolver::getMovesOrder(moves_t _moves, uint8_t* size, double offsetRad) const {
    uint8_t* order = new uint8_t[4]();
    *size = 0;

    for (int i = 0; i < 4; i++) {
        if (_moves & (1 << i))
            order[(*size)++] = i;
    }
    
    // Sort moves according to priority
    for (int i = 0; i < *size; i++) {
        for (int j = i + 1; j < *size; j++) {
            uint8_t newIndexI = order[i]; 
            uint8_t newIndexJ = order[j];

            if (offsetRad != 0.) {
                moves_t newDirectionI = order[i];
                moves_t newDirectionJ = order[j];

                // I will not explain...
                newDirectionI = radiansToDirection(
                        directionToRadians((CompassDir)(1 << order[i]))
                        + (2*PI_d - offsetRad)
                        );

                newDirectionJ = radiansToDirection(
                        directionToRadians((CompassDir)(1 << order[j]))
                        + (2*PI_d - offsetRad)
                        );

                newIndexI = dirToIndex((CompassDir)newDirectionJ);
                newIndexJ = dirToIndex((CompassDir)newDirectionI);
            } 

            if (m_movePriority[newIndexI] > m_movePriority[newIndexJ]) {
                // swap
                uint8_t temp = order[i];
                order[i] = order[j];
                order[j] = temp;
            }
        }
    }

    return order;
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

void MazeSolver::setExplorationMode(bool toggle) {
    m_explorationMode = toggle;
}

void MazeSolver::setCurrPos(const vec2<double>& pos) {
    m_currPos = pos;
    vec2<int> posR = roundPos(pos);
    m_visitedMatrix[posR.x][posR.y] = 1;
}

const vec2<int>& MazeSolver::getStartPos() const {
    return m_startPos;
} 

const vec2<int>& MazeSolver::getEndPos() const {
    return m_endPos;
}

const vec2<double>& MazeSolver::getCurrPos() const {
    return m_currPos;
} 

// Parameters:
// - pos: The current position in the maze grid, represented as a vec2<double>. The more precise the position the better.
// - distance: The distance to the wall in centimeters (double). This value is used
//   to calculate the position of the wall based on the direction.
// - dir: The direction from the current position in which the wall is located, 
//   represented by the CompassDir enum (e.g., North, South, East, West). Where north goes towards y = 0 and West goes to x = 0
void MazeSolver::markWall(const vec2 <double>& pos, const double distance, const CompassDir dir) {
    setCurrPos(pos);

    vec2<double> wallPosCm = posToCm(pos) + (vec2<double>{ distance, distance }*getDirOffset(dir));
    vec2<int> wallPosEx = posToPosEx(cmToPos(wallPosCm));

    // walls are only on even spots
    if (!(wallPosEx.x % 2 == 0 || wallPosEx.y % 2 == 0))
        return;

    m_wallMatrix[wallPosEx.x][wallPosEx.y] = 1;
}

// Parameters:
// - pos: The current position in the maze grid, represented as a vec2<double>. The more precise the position the better.
// - distance: The distance to the wall in centimeters (double). This value is used
//   to calculate the position of the wall based on the direction.
// - angleRad: The compass angle from the center of the car, 0 is North, 1/2pi is East, pi is South, 3/2pi is West. 
//             The angle is not relative to the car!
void MazeSolver::markWall(const vec2<double>& pos, const double distance, const double angleRad) {
    setCurrPos(pos);

    vec2<double> wallPosCm = posToCm(pos) + (vec2<double>{ distance*sin(angleRad), distance*cos(angleRad) });
    vec2<int> wallPosEx = posToPosEx(cmToPos(wallPosCm));

    // walls are only on even spots
    if (!(wallPosEx.x % 2 == 0 || wallPosEx.y % 2 == 0))
        return;

    m_wallMatrix[wallPosEx.x][wallPosEx.y] = 1;

}

void MazeSolver::floodFill(FixedDeque<FloodFillNode>& queue) {
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


void MazeSolver::floodFillUnvisited() {
    clearDistanceMatrix();
    FixedDeque<FloodFillNode> queue(m_MazeWidth*m_MazeHeight);
    for (int x = 0; x < m_MazeWidth; x++) {
        for (int y = 0; y < m_MazeHeight; y++) {
           if (m_visitedMatrix[x][y])
               continue;

           queue.push_back({{x,y}, 0});
           m_distanceMatrix[x][y] = 0;

        }
    }

    floodFill(queue);
}

void MazeSolver::floodFillBlind() {
    clearDistanceMatrix();
    FixedDeque<FloodFillNode> queue(m_MazeWidth * m_MazeHeight);


    // Add all possible exits
    for (int x = m_topLeft.x; x <= m_bottomRight.x; ++x) {
        for (int y = m_topLeft.y; y <= m_bottomRight.y; ++y) {
            // Skip non-boundary cells
            if (x != m_topLeft.x && x != m_bottomRight.x && y != m_topLeft.y && y != m_bottomRight.y)
                continue;

            if (m_currPos.x == x && m_currPos.y == y) {
                m_atExit = true;
            }
            vec2<int> posEx = posToPosEx(vec2<int>{x, y});

            // Check if this boundary cell is connected to the outside
            if ((x == m_topLeft.x && m_wallMatrix[posEx.x - 1][posEx.y] != 1) ||  // Left
                (x == m_bottomRight.x && m_wallMatrix[posEx.x + 1][posEx.y] != 1) || // Right
                (y == m_topLeft.y && m_wallMatrix[posEx.x][posEx.y - 1] != 1) || // Up
                (y == m_bottomRight.y && m_wallMatrix[posEx.x][posEx.y + 1] != 1)) { // Down
                queue.push_back({{x, y}, 0});
                m_distanceMatrix[x][y] = 0;
                continue;
            }

            if (m_currPos.x == x && m_currPos.y == y) {
                m_atExit = false;
            }
        }
    }

    floodFill(queue);
}


bool MazeSolver::atExit() {
    return m_atExit;
}

void MazeSolver::floodFill(const vec2<int>& destination) {
    clearDistanceMatrix();
    m_distanceMatrix[destination.x][destination.y] = 0;

    FixedDeque<FloodFillNode> queue(m_MazeWidth * m_MazeHeight);
    queue.push_back({ destination, 0 });

    floodFill(queue);
}

vec2<int> MazeSolver::getDirOffset(const CompassDir dir) const {
    return m_directions[dirToIndex(dir)];
}

double MazeSolver::directionToRadians(CompassDir dir) const {
    if (dir == North)
        return 0.;
    if (dir == South)
        return PI_d;
    if (dir == East)
        return 0.5*PI_d;
    if (dir == West)
        return 1.5*PI_d;

    return INVALID_ANGLE;
}

CompassDir MazeSolver::radiansToDirection(double angleRad) const {
    angleRad = fmod(angleRad, 2 * PI_d);
    if (angleRad < 0) angleRad += 2*PI_d;

    if (angleRad >= 7.0 / 4.0 * PI_d || angleRad < 1.0 / 4.0 * PI_d) return North;
    if (angleRad >= 1.0 / 4.0 * PI_d && angleRad < 3.0 / 4.0 * PI_d) return East;
    if (angleRad >= 3.0 / 4.0 * PI_d && angleRad < 5.0 / 4.0 * PI_d) return South;
    return West;
}

bool MazeSolver::findBounds() {
    for (int x = 0; x < m_MazeWidthEx; x++) {
        for (int y = 0; y < m_MazeWidthEx; y++) {
            if (m_wallMatrix[x][y] != 1)
                continue;

            vec2<int> pos = posExToPos({x,y});
            vec2<int> cellPosEx = posToPosEx(pos);
            
            // left
            if (pos.x < m_topLeft.x && m_wallMatrix[cellPosEx.x-1][cellPosEx.y])
                m_topLeft.x = pos.x;
            
            // up
            if (pos.y < m_topLeft.y && m_wallMatrix[cellPosEx.x][cellPosEx.y-1])
                m_topLeft.y = pos.y;
            
            // right
            if (pos.x > m_bottomRight.x && m_wallMatrix[cellPosEx.x+1][cellPosEx.y])
                m_bottomRight.x = pos.x;
            
            // bottom
            if (pos.y > m_bottomRight.y && m_wallMatrix[cellPosEx.x][cellPosEx.y+1])
                m_bottomRight.y = pos.y;
            
            if (m_bottomRight.x == -1 || m_bottomRight.y == -1 || m_topLeft.x == 999 || m_topLeft.y == 999)
                continue;

            if (((m_bottomRight.x - m_topLeft.x + 1) >= ((m_MazeWidth + 1) / 2)) &&
                ((m_bottomRight.y - m_topLeft.y + 1) >= ((m_MazeHeight + 1) / 2))) {
                m_blindStage = Stage::FOLLOW_SIDES;
                return true;
            }

        }
    }
    return false;
}

vec2<int> MazeSolver::getNextMove(const double carBearing) {
    static vec2<int> lastMove = roundPos(m_currPos);

    // TODO: Could optimize this to floodfill only once per currPos

    if (m_blind && m_blindStage == Stage::BOUND_SEARCH) {
        // FloodFill with all unvisited set to 0
        floodFillUnvisited();

        // Look for bounds to switch stage
        findBounds();
        
    } 

    if (m_blind && m_blindStage == Stage::FOLLOW_SIDES) {
        // FloodFill with all possible exits set to 0
        floodFillBlind();

        if (atExit())
            return m_currPos;
    } 

    moves_t moves = getPossibleMoves();

    uint8_t orderSize;
    uint8_t* order = getMovesOrder(moves, &orderSize, carBearing); 
    
    if (orderSize == 0) {
        clearWallMatrix();
    }

    vec2<int> bestMove{ -1 };
    int bestDist = 9999;
    
    for (int i = 0; i < orderSize; i++) {
        moves_t dir = moves & (1 << order[i]);

        if (!dir)
            continue;

        vec2<int> newPos = roundPos(m_currPos) + getDirOffset((CompassDir)dir);

        // Skip previous move
        if (newPos == lastMove)
            continue;

        int dist = m_distanceMatrix[newPos.x][newPos.y];
        
        bool visited = m_visitedMatrix[newPos.x][newPos.y]; 
        if (!m_explorationMode && !visited)
            continue; 

        if (dist < bestDist) {
            bestDist = dist;
            bestMove = newPos;
        }
    }

    // Check previous move as last
    if (m_distanceMatrix[lastMove.x][lastMove.y] < bestDist)
        bestMove = lastMove;

    // free memory
    delete[] order;
    
    // TODO: check if this isn't broken
    lastMove = bestMove;

    return bestMove;
}

moves_t MazeSolver::getPossibleMoves() const {
    moves_t out = 0;
    for (int i = 0; i < 4; i++) {
        vec2<int> wallPos = posToPosEx(m_currPos) + m_directions[i];
        if (m_wallMatrix[wallPos.x][wallPos.y] == 1)
            continue;

        out |= (1 << i);
    }

    return out;
}


void MazeSolver::printWalls() const {
#ifdef DEBUG
    for (int y = 0; y < m_MazeHeightEx; y++) {
        for (int x = 0; x < m_MazeWidthEx; x++) {
            // part of normal grid
            if (x % 2 != 0 && y % 2 != 0) {
                vec2<int> pos = posExToPos({x,y});
                if (m_visitedMatrix[pos.x][pos.y]) {
                    std::cout << "+ ";
                    continue;
                }

                if (m_distanceMatrix[pos.x][pos.y] == -1) {
                    std::cout << "X ";
                    continue;
                }

                std::cout << "~ ";
                continue;
            }
            
            if (m_wallMatrix[x][y]) {
                std::cout << "# ";
                continue;
            }


            if (x % 2 == 0 && y % 2 == 0) {
                std::cout << ". ";
                continue;
            }

            std::cout << "  ";
            
        }
        std::cout << "\n";
    }
    std::cout << "LEGEND: #: Wall, ~: Accesible, X: Inaccesible, +: Visited\n";
#endif
}

void MazeSolver::printDists() const {
#ifdef DEBUG
    for (int y = 0; y < m_MazeHeight; y++) {
        for (int x = 0; x < m_MazeWidth; x++) {
            std::cout << std::setw(3) << (int)(m_distanceMatrix[x][y]) << " ";
        }
        std::cout << "\n";
    }
#endif
}
