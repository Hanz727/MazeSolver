#include "MazeSolver.h"
#include "FixedDeque.h"

#include <math.h>

#ifdef DEBUG
#include <iostream>
#include <iomanip>
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
        endPos
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

uint8_t* MazeSolver::getMovesOrder(moves_t _moves, uint8_t* size) const {
    uint8_t* order = new uint8_t[4]();
    *size = 0;

    for (int i = 0; i < 4; i++) {
        if (_moves & (1 << i))
            order[(*size)++] = i;
    }
    
    // Sort moves according to priority
    for (int i = 0; i < *size; i++) {
        for (int j = i + 1; j < *size; j++) {
            if (m_movePriority[order[i]] > m_movePriority[order[j]]) {
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

void MazeSolver::floodFill(const vec2<int>& destination) {
    clearDistanceMatrix();
    m_distanceMatrix[destination.x][destination.y] = 0;

    FixedDeque<FloodFillNode> queue(m_MazeWidth * m_MazeHeight);
    queue.push_back({ {12,0}, 0 });

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

CompassDir MazeSolver::radiansToDirection(double angleRad) const {
    constexpr double PI_4 = PI_d / 4.;

    angleRad = fmod(angleRad, 2 * PI_d);
    if (angleRad < 0) angleRad += 2*PI_d;

    if (angleRad >= 7.0 / 4.0 * PI_d || angleRad < 1.0 / 4.0 * PI_d) return North;
    if (angleRad >= 1.0 / 4.0 * PI_d && angleRad < 3.0 / 4.0 * PI_d) return East;
    if (angleRad >= 3.0 / 4.0 * PI_d && angleRad < 5.0 / 4.0 * PI_d) return South;
    return West;}

vec2<int> MazeSolver::getNextMove(const double carBearing) const {
    static vec2<int> lastMove = roundPos(m_currPos);
    moves_t moves = getPossibleMoves();

    // CANNOT USE RIGHT WALL HUG FOR BOUND SEARCH!
    // TODO: Create some other algorithm for bound search
    if (m_blind && m_blindStage == Stage::BOUND_SEARCH) {
        moves_t right = radiansToDirection(carBearing + 0.5 * PI_d);
        moves_t forward = radiansToDirection(carBearing);
        moves_t left = radiansToDirection(carBearing - 0.5 * PI_d);
        moves_t backward = radiansToDirection(carBearing + PI_d);
        
        vec2<int> bestMovePos;
        moves_t bestMoveDir = 0;
        
        // ugly but works
        for (int i = 0; i < 4; i++) {
            moves_t dir = moves & (1 << i);
            if (!dir)
                continue;

            // always go right if possible
            if (dir == right) {
                bestMovePos = roundPos(m_currPos) + getDirOffset((CompassDir)dir);
                bestMoveDir = dir;
                break;
            }
            
            if (dir == forward) {
                bestMovePos = roundPos(m_currPos) + getDirOffset((CompassDir)dir);
                bestMoveDir = dir;
                continue;
            }
            
            if (dir == left && bestMoveDir != forward) {
                bestMovePos = roundPos(m_currPos) + getDirOffset((CompassDir)dir);
                bestMoveDir = dir;
                continue;
            }

            if (dir == backward && bestMoveDir == 0) {
                bestMovePos = roundPos(m_currPos) + getDirOffset((CompassDir)dir);
                continue;
            }

        }

        lastMove = bestMovePos;
        return bestMovePos;
    }

    if (m_blind && m_blindStage == Stage::FOLLOW_SIDES) {
        // Call blind floodFill
    } 

    uint8_t orderSize;
    uint8_t* order = getMovesOrder(moves, &orderSize); 

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
