#include "MazeSolver.h"
#include <string.h>
#include "pch.h"

MazeSolver::MazeSolver(
    const double wallWidth,
    const double cellWidth,
    const double cellHeight,
    const int8_t mazeWidth,
    const int8_t mazeHeight,
    const vec2<int8_t> startPos,
    const vec2<int8_t> endPos,
    const bool blind 
)
{

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
        const int8_t mazeWidth,
        const int8_t mazeHeight,
        const vec2<int8_t> startPos,
        const vec2<int8_t> endPos,
        const bool blind
) {

    m_wallWidth = wallWidth;
    m_cellWidth = cellWidth;
    m_cellHeight = cellHeight;
    m_mazeWidth = mazeWidth;
    m_mazeHeight = mazeHeight;
    m_startPos = startPos;
    m_endPos = endPos;
    m_blind = blind;
    
    m_mazeHeightEx = mazeHeight * 2 + 1;
    m_mazeWidthEx = mazeWidth * 2 + 1;

    assert(mazeHeight <= 32);
    assert(mazeWidth <= 32);
    
    m_currPos = m_startPos;
    m_visitedMatrix[m_startPos.x][m_startPos.y] = 1;

    clearDistanceMatrix();
    clearWallMatrix();
}

void MazeSolver::clearDistanceMatrix() {
    // Warning memset only works properly for 0 and -1.
    memset(m_distanceMatrix, -1, sizeof(m_distanceMatrix));
}

void MazeSolver::clearWallMatrix() {
    memset(m_wallMatrix, 0, sizeof(m_wallMatrix));
    if (m_blind)
        return;

    for (int8_t x = 0; x < m_mazeWidthEx; x++) {
        for (int8_t y = 0; y < m_mazeHeightEx; y++) {
            if (x != m_mazeWidthEx - 1 && y != m_mazeHeightEx - 1 && x != 0 && y != 0)
                continue;

            m_wallMatrix[x][y] = 1;
        }
    }
}


vec2<int8_t> MazeSolver::projectPos(const vec2<double>& pos, double distance, double angleRad) const {
    assert(pos.x >= 0 && pos.y >= 0);
    assert(pos.x < m_mazeWidth && pos.y < m_mazeHeight);
    assert(distance > 0);
    return roundPos(cmToPos(posToCm(pos) + vec2<double>{distance*sin(angleRad), distance*cos(angleRad)}));
}

void MazeSolver::setMovePriority(int8_t priority[4]) {
    for (int8_t i = 0; i < 4; i++) {
        m_movePriority[i] = priority[i];
    }
}

int8_t MazeSolver::dirToIndex(CompassDir dir) const {
    if (dir == North)
        return 0;
    if (dir == South)
        return 1;
    if (dir == East)
        return 2;
    return 3;
}

int8_t* MazeSolver::getMovesOrder(directionFlags moves, int8_t* size, double offsetRad) const {
    assert(moves <= 15);
    assert(size != nullptr);

    int8_t* order = new int8_t[4]();
    *size = 0;

    for (int8_t i = 0; i < 4; i++) {
        if (moves & (1 << i))
            order[(*size)++] = i;
    }
    
    // Sort moves according to priority
    for (int8_t i = 0; i < *size; i++) {
        for (int8_t j = i + 1; j < *size; j++) {
            int8_t newIndexI = order[i]; 
            int8_t newIndexJ = order[j];

            if (offsetRad != 0.) {
                CompassDir newDirectionI;
                CompassDir newDirectionJ;

                newDirectionI = radiansToDirection(
                        directionToRadians((CompassDir)(1 << order[i]))
                        + (2*PI_d - offsetRad)
                );

                newDirectionJ = radiansToDirection(
                        directionToRadians((CompassDir)(1 << order[j]))
                        + (2*PI_d - offsetRad)
                );

                newIndexI = dirToIndex(newDirectionI);
                newIndexJ = dirToIndex(newDirectionJ);
            } 

            if (m_movePriority[newIndexI] > m_movePriority[newIndexJ]) {
                // swap
                int8_t temp = order[i];
                order[i] = order[j];
                order[j] = temp;
            }
        }
    }

    return order;
}

vec2<double> MazeSolver::posToCm(const vec2<double>& pos) const {
    assert(pos.x >= 0 && pos.y >= 0);
    assert(pos.x < m_mazeWidth && pos.y < m_mazeHeight);    

    return vec2<double>(
        (m_cellWidth + m_wallWidth) * pos.x + (m_wallWidth + m_cellWidth / 2.),
        (m_cellHeight + m_wallWidth) * pos.y + (m_wallWidth + m_cellHeight / 2.)
    );
}

vec2<double> MazeSolver::cmToPos(const vec2<double>& cm) const {
    assert(cm.x >= 0 && cm.y >= 0);

    return vec2<double>(
        (cm.x - m_wallWidth - m_cellWidth / 2.) / (m_cellWidth + m_wallWidth),
        (cm.y - m_wallWidth - m_cellHeight / 2.) / (m_cellHeight + m_wallWidth)
    );
}

vec2<int8_t> MazeSolver::roundPos(const vec2<double>& pos) const {
    assert(pos.x >= 0 && pos.y >= 0);
    assert(pos.x < m_mazeWidth && pos.y < m_mazeHeight);    

    return vec2<int8_t>{(int8_t)round(pos.x), (int8_t)round(pos.y)};
}

vec2<int8_t> MazeSolver::posToPosEx(const vec2<double>& pos) const {
    assert(pos.x >= -0.5 && pos.y >= -0.5);
    assert(pos.x < m_mazeWidth && pos.y < m_mazeHeight);    

    return vec2<int8_t>(
        round(pos.x * 2. + 1.),
        round(pos.y * 2. + 1.)
    );
}

vec2<double> MazeSolver::posExToPos(const vec2<int8_t>& posEx) const {
    assert(posEx.x >= 0 && posEx.y >= 0);
    assert(posEx.x < m_mazeWidthEx && posEx.y < m_mazeHeightEx);    

    return vec2<double>(
        (posEx.x - 1.) / 2.,
        (posEx.y - 1.) / 2.
    );
}

void MazeSolver::setExplorationMode(bool toggle) {
    m_explorationMode = toggle;
}

void MazeSolver::setCurrPos(const vec2<double>& pos) {
    assert(pos.x >= 0 && pos.y >= 0);
    assert(pos.x < m_mazeWidth && pos.y < m_mazeHeight);    
    assert((abs(m_currPos.x - pos.x) + abs(m_currPos.y-pos.y)) <= 1.01);

    m_currPos = pos;
    vec2<int> posR = roundPos(pos);
    m_visitedMatrix[posR.x][posR.y] = 1;
}

const vec2<int8_t>& MazeSolver::getStartPos() const {
    return m_startPos;
} 

const vec2<int8_t>& MazeSolver::getEndPos() const {
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
void MazeSolver::markWall(const vec2 <double>& pos, double distance, CompassDir dir) {
    assert(pos.x >= 0 && pos.y >= 0);
    assert(pos.x < m_mazeWidth && pos.y < m_mazeHeight);    
    //setCurrPos(pos);

    vec2<double> wallPosCm = posToCm(pos) + (vec2<double>{ distance, distance }*getDirOffset(dir));
    vec2<int8_t> wallPosEx = posToPosEx(cmToPos(wallPosCm));

    // walls are only on even spots
    if (!(wallPosEx.x % 2 == 0 || wallPosEx.y % 2 == 0))
        return;

    if (wallPosEx.x < 0 || wallPosEx.y < 0)
        return;

    m_wallMatrix[wallPosEx.x][wallPosEx.y] = 1;
}

// Parameters:
// - pos: The current position in the maze grid, represented as a vec2<double>. The more precise the position the better.
// - distance: The distance to the wall in centimeters (double). This value is used
//   to calculate the position of the wall based on the direction.
// - angleRad: The compass angle from the center of the car, 0 is North, 1/2pi is East, pi is South, 3/2pi is West. 
//             The angle is not relative to the car!
void MazeSolver::markWall(const vec2<double>& pos, double distance, double angleRad) {
    assert(pos.x >= 0 && pos.y >= 0);
    assert(pos.x < m_mazeWidth && pos.y < m_mazeHeight);    
    //setCurrPos(pos);

    vec2<double> wallPosCm = posToCm(pos) + (vec2<double>{ distance*sin(angleRad), distance*cos(angleRad) });
    vec2<int8_t> wallPosEx = posToPosEx(cmToPos(wallPosCm));

    // walls are only on even spots
    if (!(wallPosEx.x % 2 == 0 || wallPosEx.y % 2 == 0))
        return;

    if (wallPosEx.x < 0 || wallPosEx.y < 0)
        return;

    m_wallMatrix[wallPosEx.x][wallPosEx.y] = 1;

}

void MazeSolver::floodFill(FixedDeque<FloodFillNode>& queue) {
    while (!(queue.is_empty())) {
        FloodFillNode node = queue.pop_front();

        for (vec2<int8_t> dir : m_directions) {
            vec2<int8_t> newPos = node.pos + dir;

            // Skip OOB
            if (newPos.x < 0 || newPos.y < 0 || newPos.x >= m_mazeWidth || newPos.y >= m_mazeHeight)
                continue;

            // Skip visited
            if (m_distanceMatrix[newPos.x][newPos.y] != -1)
                continue;

            vec2<int8_t> newPosEx = posToPosEx(newPos);
            vec2<int8_t> wallPos = newPosEx - dir; // step back

            // Can't drive through walls
            if (m_wallMatrix[wallPos.x][wallPos.y] == 1)
                continue;

            m_distanceMatrix[newPos.x][newPos.y] = node.dist + 1;
            queue.push_back({ newPos, (int16_t)(node.dist + 1) });
        }
    }   
}


void MazeSolver::floodFillUnvisited() {
    clearDistanceMatrix();
    FixedDeque<FloodFillNode> queue(m_mazeWidth*m_mazeHeight);
    for (int8_t x = 0; x < m_mazeWidth; x++) {
        for (int8_t y = 0; y < m_mazeHeight; y++) {
           if (m_visitedMatrix[x][y])
               continue;

           queue.push_back({{x,y}, 0});
           m_distanceMatrix[x][y] = 0;
        }
    }

    floodFill(queue);
}

void MazeSolver::floodFillBorders() {
    clearDistanceMatrix();
    FixedDeque<FloodFillNode> queue(m_mazeWidth * m_mazeHeight);

    for (int8_t x = m_topLeft.x; x <= m_bottomRight.x; ++x) {
        for (int8_t y = m_topLeft.y; y <= m_bottomRight.y; ++y) {
            // Skip non-boundary cells
            if (x != m_topLeft.x && x != m_bottomRight.x && y != m_topLeft.y && y != m_bottomRight.y)
                continue;

            if (m_currPos.x == x && m_currPos.y == y) {
                m_atExit = true;
            }
            vec2<int8_t> posEx = posToPosEx(vec2<int8_t>{x, y});

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

void MazeSolver::floodFill(const vec2<int8_t>& destination) {
    clearDistanceMatrix();
    m_distanceMatrix[destination.x][destination.y] = 0;

    FixedDeque<FloodFillNode> queue(m_mazeWidth * m_mazeHeight);
    queue.push_back({ destination, 0 });

    floodFill(queue);
}

vec2<int8_t> MazeSolver::getDirOffset(CompassDir dir) const {
    return m_directions[dirToIndex(dir)];
}

double MazeSolver::directionToRadians(CompassDir dir) const {
    if (dir == East)
        return 0.5*PI_d;
    if (dir == West)
        return 1.5*PI_d;
    if (dir == South)
        return PI_d;
    // North
    return 0.;
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
    for (int8_t x = 0; x < m_mazeWidthEx; x++) {
        for (int8_t y = 0; y < m_mazeWidthEx; y++) {
            if (m_wallMatrix[x][y] != 1)
                continue;

            vec2<double> wallPos = posExToPos({x,y});

            // left
            if (wallPos.x < m_topLeft.x && (m_currPos.x > wallPos.x))
                m_topLeft.x = round(wallPos.x);
            
            // up
            if (wallPos.y < m_topLeft.y && (m_currPos.y > wallPos.y)) 
                m_topLeft.y = round(wallPos.y);
            
            // right
            if (wallPos.x > m_bottomRight.x && (m_currPos.x < wallPos.x))
                m_bottomRight.x = (int8_t)wallPos.x; // round down
            
            // bottom
            if (wallPos.y > m_bottomRight.y && (m_currPos.y < wallPos.y))
                m_bottomRight.y = (int8_t)wallPos.y; // round down
            
            if (m_bottomRight.x == -1 || m_bottomRight.y == -1 || m_topLeft.x == 127 || m_topLeft.y == 127)
                continue;

            if (((m_bottomRight.x - m_topLeft.x + 1) >= ((m_mazeWidth + 1) / 2)) &&
                ((m_bottomRight.y - m_topLeft.y + 1) >= ((m_mazeHeight + 1) / 2))) {
                m_blindStage = Stage::EXIT_SEARCH;
                return true;
            }

        }
    }
    return false;
}

vec2<int8_t> MazeSolver::getNextMove(double carBearing) {
    static vec2<int8_t> lastMove = roundPos(m_currPos);
    static int8_t recursionCount = 0;
    
    if (m_blind) {
        if (m_blindStage == Stage::BOUND_SEARCH) {
            // Look for bounds to switch stage
            findBounds();
        } 

        // findBounds may change the stage
        if (m_blindStage == Stage::BOUND_SEARCH) {
            floodFillUnvisited();
        }

        if (m_blindStage == Stage::EXIT_SEARCH) {
            floodFillBorders();

            if (atExit()) {
                return m_currPos;
            }
        } 
    }

    directionFlags moves = getPossibleMoves();

    int8_t orderSize;
    int8_t* order = getMovesOrder(moves, &orderSize, carBearing); 
    
    if (orderSize == 0) {
        clearWallMatrix();

        // only try calling the function again once
        if (recursionCount++ == 0) {
            return getNextMove(carBearing);
        }
    }
    recursionCount = 0;

    vec2<int8_t> bestMove{ -1 };
    int16_t bestDist = 9999;
    
    for (int8_t i = 0; i < orderSize; i++) {
        CompassDir dir = (CompassDir)(moves & (1 << order[i]));
        vec2<int8_t> newPos = roundPos(m_currPos) + getDirOffset(dir);

        if (newPos == lastMove)
            continue;

        int16_t dist = m_distanceMatrix[newPos.x][newPos.y];
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

    delete[] order;
    
    lastMove = bestMove;
    return bestMove;
}

directionFlags MazeSolver::getPossibleMoves() const {
    directionFlags out = 0;
    vec2<int8_t> posEx = posToPosEx(m_currPos);
    for (int8_t i = 0; i < 4; i++) {
        vec2<int8_t> wallPos = posEx + m_directions[i];
        if (m_wallMatrix[wallPos.x][wallPos.y] == 1)
            continue;

        out |= (1 << i);
    }

    return out;
}


void MazeSolver::printWalls() const {
#ifdef DEBUG
    for (int8_t y = 0; y < m_mazeHeightEx; y++) {
        for (int8_t x = 0; x < m_mazeWidthEx; x++) {
            // part of normal grid
            if (x % 2 != 0 && y % 2 != 0) {
                vec2<int8_t> pos = posExToPos({x,y});
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
    for (int8_t y = 0; y < m_mazeHeight; y++) {
        for (int8_t x = 0; x < m_mazeWidth; x++) {
            std::cout << std::setw(3) << (int)(m_distanceMatrix[x][y]) << " ";
        }
        std::cout << "\n";
    }
#endif
}
