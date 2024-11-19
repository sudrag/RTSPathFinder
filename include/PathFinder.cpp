// Local lib includes
#include "PathFinder.hpp"
#include "PathFinderConstants.hpp"

// External lib includes
#include <nlohmann/json.hpp> // Include nlohmann JSON library

// Standard Includes
#include <fstream>
#include <iostream>
#include <queue>
#include <stdexcept>
#include <unordered_set>
using json = nlohmann::json;
using namespace PathPlanner;
using Position = PathPlanner::PathFinder::Position;

/**
 * @brief Constructor for the PathFinder Class. Parses config file and map file
 */
PathFinder::PathFinder(const std::string &configFilePath)
    : m_targetPositions({}), m_startPositions({}), m_map({})
{
    parseConfig(configFilePath);
    parseMap(m_mapFilePath);
}

/**
 * @brief Destructor for the PathFinder Class. Clears all the member variables
 */
PathFinder::~PathFinder()
{
    m_targetPositions.clear();
    m_startPositions.clear();
    m_map.clear();
    m_terrainKeys.clear();
    m_mapFilePath.clear();
}

/**
 * @brief Parse the config file to obtain map file path and obtain terrain values for start, target,
 * elevated and reachable.
 *
 * @param configFile Config File Path declared by the constructor
 *
 */
void PathFinder::parseConfig(const std::string &configFile)
{
    try
    {
        std::cout << "Parsing config file" << std::endl;
        std::ifstream file(configFile);
        if (!file.is_open())
        {
            std::cerr << "JSON parsing error at file: " << __FILE__ << ", line: " << __LINE__
                      << std::endl;
            throw std::runtime_error("Failed to open config file: " + configFile);
        }

        json configJson = json::parse(file);

        // Parse terrain keys
        if (configJson.contains(TerrainKeys))
        {
            for (const auto &[key, value] : configJson[TerrainKeys].items())
            {
                m_terrainKeys[key] = value;
            }
        }
        else
        {
            std::cerr << "JSON parsing error at file: " << __FILE__ << ", line: " << __LINE__
                      << std::endl;
            throw std::runtime_error("Map file path not specified in config.");
        }

        // Required terrain keys
        const std::vector<std::string> requiredKeys = {Start, Target, Elevated, Reachable};

        // Validate all required keys are present
        for (const auto &key : requiredKeys)
        {
            if (m_terrainKeys.find(key) == m_terrainKeys.end())
            {
                std::cerr << "JSON parsing error at file: " << __FILE__ << ", line: " << __LINE__
                          << std::endl;
                throw std::runtime_error("Missing required terrain key: " + key);
            }
        }

        // Validate uniqueness of terrain keys
        std::unordered_set<int> keyValues;
        for (const auto &key : requiredKeys)
        {
            int value = m_terrainKeys[key];
            if (keyValues.count(value))
            {
                std::cerr << "JSON parsing error at file: " << __FILE__ << ", line: " << __LINE__
                          << std::endl;
                throw std::runtime_error("Duplicate terrain key value detected for: " + key);
            }
            keyValues.insert(value);
        }

        // Parse map file path
        if (configJson.contains(MapFile))
        {
            m_mapFilePath = configJson.at(MapFile).get<std::string>();
        }
        else
        {
            std::cerr << "JSON parsing error at file: " << __FILE__ << ", line: " << __LINE__
                      << std::endl;
            throw std::runtime_error("Map file path not specified in config.");
        }
    }
    catch (const nlohmann::json::exception &e)
    {
        std::cerr << "JSON parsing error: " << e.what() << "\n"
                  << "Error occurred at file: " << __FILE__ << ", line: " << __LINE__ << std::endl;
        throw std::runtime_error("JSON Error in parseConfig function");
    }
    catch (const std::exception &e)
    {
        // Catch any other standard exceptions
        std::cerr << "Error: " << e.what() << "\n"
                  << "Error occurred at file: " << __FILE__ << ", line: " << __LINE__ << std::endl;
        throw std::runtime_error("STD Error in parseConfig function.");
    }
}

/**
 * @brief Parse the map file specified by the config file. Parse map to identify all the start and
 * target positions in the maps
 *
 * @param mapFile File path to the map file
 *
 */
void PathFinder::parseMap(const std::string &mapFile)
{
    try
    {
        std::cout << "Parsing map data file" << std::endl;
        std::ifstream file(mapFile);
        if (!file.is_open())
        {
            std::cerr << "JSON parsing error at file: " << __FILE__ << ", line: " << __LINE__
                      << std::endl;
            throw std::runtime_error("Failed to open map file: " + mapFile);
        }

        json mapJson;
        file >> mapJson;
        int width, height;

        if (mapJson.contains(Tilesets))
        {
            width = mapJson[Tilesets][0][TileWidth];

            height = mapJson[Tilesets][0][TileHeight];

            // Convert 1D map data to 2D
            m_map.resize(height, std::vector<int>(width));
        }
        else
        {
            std::cerr << "JSON parsing error at file: " << __FILE__ << ", line: " << __LINE__
                      << std::endl;
            throw std::runtime_error("Tilesets not found in JSON");
        }

        // Check if the key "Layers" exists
        if (mapJson.contains(Layers))
        {
            const auto &layers = mapJson[Layers];

            // Check if the first layer exists and is an array
            if (!layers.empty() && layers[0].contains(Data))
            {
                const auto &data = layers[0][Data];
                // Process the data here
                std::cout << "Data found in first layer!" << std::endl;

                for (int i = 0; i < height; ++i)
                {
                    for (int j = 0; j < width; ++j)
                    {
                        Position startPosition, targetPosition;
                        int index = i * width + j;
                        m_map[i][j] = int(data[index]);
                        if (m_map[i][j] == m_terrainKeys[Start])
                        {
                            startPosition = {i, j};
                            m_startPositions.push_back(startPosition);
                            std::cout << "Start Position " << i << " ," << j << std::endl;
                        }
                        else if (m_map[i][j] == m_terrainKeys[Target])
                        {
                            targetPosition = {i, j};
                            m_targetPositions.push_back(targetPosition);
                            std::cout << "Target Position " << i << " ," << j << std::endl;
                        }
                    }
                }
            }
            else{
                std::cerr << "JSON parsing error at file: " << __FILE__ << ", line: " << __LINE__
                      << std::endl;
            throw std::runtime_error("Missing data field in Layer");
            }
        }
        else
        {
            std::cerr << "JSON parsing error at file: " << __FILE__ << ", line: " << __LINE__
                      << std::endl;
            throw std::runtime_error("Missing Layer field in Map");
        }

        validateMapPositions();
        std::cout << "Map is parsed" << std::endl;
        printMap();
    }
    catch (const nlohmann::json::exception &e)
    {
        std::cerr << "JSON parsing error: " << e.what() << "\n"
                  << "Error occurred at file: " << __FILE__ << ", line: " << __LINE__ << std::endl;
        throw std::runtime_error("JSON Error in parseMap function");
    }
    catch (const std::exception &e)
    {
        // Catch any other standard exceptions
        std::cerr << "Error: " << e.what() << "\n"
                  << "Error occurred at file: " << __FILE__ << ", line: " << __LINE__ << std::endl;
        throw std::runtime_error("STD Error in parseMap function.");
    }
}

/**
 * @brief Validate start and target positions specified on the map. Check for number of each and if
 * any of them are already on an obstacle
 *
 */
void PathFinder::validateMapPositions()
{
    size_t len1 = m_startPositions.size();
    size_t len2 = m_targetPositions.size();

    if (len1 > len2)
    {
        // If there are lesser target positions than starting positions , duplicate the last target
        // position till it matches the start positions size
        auto lastTarget = m_targetPositions.back();
        m_targetPositions.insert(m_targetPositions.end(), len1 - len2, lastTarget);
    }
    else if (len1 < len2)
    {
        // If there are lesser starting positions than target positions, trim the start positions
        // vector to have 1:1 co-relation
        m_targetPositions.resize(len1);
    }
}

/**
 * @brief Allows calling applications to access Start position of particular unit
 *
 * @param index Access start position of that particular unit
 *
 */
Position PathFinder::GetStartPosition(int index) const
{
    if (index >= 0 && index < m_startPositions.size())
    {
        return m_startPositions[index];
    }
    else
    {
        throw std::out_of_range("Index out of bounds");
    }
}

/**
 * @brief Allows calling applications to access Target position of particular unit
 *
 * @param index Access target position of that particular unit
 *
 */
Position PathFinder::GetTargetPosition(int index) const
{
    if (index >= 0 && index < m_targetPositions.size())
    {
        return m_targetPositions[index];
    }
    else
    {
        throw std::out_of_range("Index out of bounds");
    }
}

/**
 * @brief Checks if the given Position exists within the bounds of a map and is reachable
 *
 * @param pos is an instance of the Position struct used to defined x and y positions in a 2D Map
 *
 * @return bool true if position is valid, false if position is out of bounds or on an obstactle
 *
 */
bool PathFinder::isValidPosition(const Position &pos) const
{
    const int elevated = m_terrainKeys.at(Elevated);
    return pos.x >= 0 && pos.x < m_map.size() && pos.y >= 0 && pos.y < m_map[0].size() &&
           m_map[pos.x][pos.y] != elevated;
}

/**
 * @brief Compute the Manhattan distance between two points in a two dimensional map. Used as the
 * heuristic for the A* algorithm
 *
 * @param a,b are both instances of the Position struct providing the two positions in the 2D Map
 *
 * @return int providing the Manhattan distance between the two points
 */
int PathFinder::manhattanDistance(Position a, Position b) const
{
    return abs(a.x - b.x) + abs(a.y - b.y);
}

/**
 * @brief Used to identify if a position on the map has collision with any of the other units
 *
 * @param positions is a vector of current position of all the units.
 *        newPosition is the target position being checked for collision. This is generally one of
 * the neighbors of a chosen node currentIndex of the unit being checked for. Used to skip for
 * checking collision against the same unit in the iteration
 *
 * @return bool return true if collision is detected with another unit, return false if no collision
 * is detected
 *
 */
bool PathFinder::hasCollision(const std::vector<Position> &positions, const Position &newPosition,
                              size_t currentIndex) const
{
    for (size_t i = 0; i < positions.size(); ++i)
    {
        if (i != currentIndex && positions[i] == newPosition)
        {
            return true;
        }
    }
    return false;
}

/**
 * @brief Find Paths is the core logic of this class. It implements A* search algorithm for all
 * units in the map. It ensures each unit moves by one step in each iteration and paths are updates
 * till all units reach their targets. It reconstructs the paths after the units have reached their
 * targets
 *
 */
void PathFinder::FindPaths()
{
    // Initialize current positions for all units
    std::vector<std::priority_queue<Node, std::vector<Node>, std::greater<Node>>> openLists(
        m_startPositions.size());
    std::vector<std::unordered_map<Position, Node>> allNodes(m_startPositions.size());
    std::vector<std::unordered_set<Position>> closedLists(m_startPositions.size());
    std::vector<std::vector<Position>> paths(
        m_startPositions.size()); // Vector of paths for each unit
    std::vector<bool> reachedTargets(m_startPositions.size(),
                                     false); // Track which units have reached their targets
    std::vector<Position> currentPositions =
        m_startPositions; // Track current positions of all units

    // Initialize each unit's open list with its start node
    for (size_t i = 0; i < m_startPositions.size(); ++i)
    {
        Node startNode(m_startPositions[i], 0,
                       manhattanDistance(m_startPositions[i], m_targetPositions[i]), nullptr);
        allNodes[i][m_startPositions[i]] = startNode;
        openLists[i].push(startNode);
    }

    bool allReached = false;

    while (!allReached)
    {
        allReached = true;

        for (size_t i = 0; i < m_startPositions.size(); ++i)
        {
            if (reachedTargets[i])
            {
                // Skip this robot as it has already reached its target
                continue;
            }

            if (openLists[i].empty())
            {
                continue;
            }

            Node currentNode = openLists[i].top();
            openLists[i].pop();

            if (currentNode.pos == m_targetPositions[i])
            {
                // Goal reached, reconstruct path
                std::vector<Position> path;
                for (const Node *node = &currentNode; node != nullptr; node = node->parent)
                {
                    path.push_back(node->pos);
                }
                std::reverse(path.begin(), path.end());
                paths[i] = path;

                std::cout << "Unit " << i << " has reached its target." << std::endl;
                reachedTargets[i] = true;
                continue;
            }

            // Mark as visited
            closedLists[i].insert(currentNode.pos);

            std::vector<Position> neighbors = getNeighborsforCurrentNode(currentNode);

            for (const auto &neighbor : neighbors)
            {
                // Skip invalid or already visited positions
                if (!isValidPosition(neighbor) || closedLists[i].count(neighbor) ||
                    hasCollision(currentPositions, neighbor, i))
                {
                    continue;
                }

                int gCost = currentNode.gCost + 1;
                int hCost = manhattanDistance(neighbor, m_targetPositions[i]);
                Node neighborNode(neighbor, gCost, hCost, &allNodes[i][currentNode.pos]);

                // Add new nodes or update existing ones if a better path is found
                if (!allNodes[i].count(neighbor) || gCost < allNodes[i][neighbor].gCost)
                {
                    allNodes[i][neighbor] = neighborNode;
                    openLists[i].push(neighborNode);
                }
            }

            // Update current position for collision detection
            currentPositions[i] = currentNode.pos;

            // Not all units have reached their targets yet
            if (!reachedTargets[i])
            {
                allReached = false;
            }
        }
    }

    printPaths(paths);
    printMap(paths);
}

/**
 * @brief Used to print all the solved paths for the map
 *
 */
void PathFinder::printPaths(const std::vector<std::vector<Position>> &paths) const
{
    for (size_t i = 0; i < paths.size(); ++i)
    {
        if (!paths[i].empty())
        {
            std::cout << "Path for unit " << i << ":" << std::endl;
            for (const auto &pos : paths[i])
            {
                std::cout << "(" << pos.x << ", " << pos.y << ") ";
            }
            std::cout << std::endl;
        }
        else
        {
            std::cerr << "No valid path found for unit " << i << std::endl;
        }
    }
}

/**
 * @brief Used to identify all the neighboring nodes the unit can move from its current node
 *
 * @param currentNode Current node the unit is in
 *
 * @return vector<Position> all the viable positions the unit can move to from the current node
 *
 */
std::vector<Position> PathFinder::getNeighborsforCurrentNode(Node currentNode)
{
    return {{currentNode.pos.x + 1, currentNode.pos.y},
            {currentNode.pos.x - 1, currentNode.pos.y},
            {currentNode.pos.x, currentNode.pos.y + 1},
            {currentNode.pos.x, currentNode.pos.y - 1}};
}

/**
 * @brief Function used to print the map with all the start , target, reachable and elevated
 * positions with unique symbols. Function also prints all the solved paths for each unit. Each
 * unit's path has a unique color
 *
 * @param paths Solved paths for all the units in the map. Arguement has a default empty value to
 * reuse the function when printing the map after parsing but prior to solving
 *
 */
void PathFinder::printMap(const std::vector<std::vector<Position>> &paths) const
{
    int rows = m_map.size();
    int cols = m_map[0].size();

    for (int x = 0; x < rows; ++x)
    {
        for (int y = 0; y < cols; ++y)
        {
            Position current{x, y};
            bool isPath = false;
            bool isStart = false;
            bool isTarget = false;
            int colorCode;

            // Check if the current position is a start or target position for any unit
            for (size_t i = 0; i < m_startPositions.size(); ++i)
            {
                if (current == m_startPositions[i])
                {
                    isStart = true;
                    break;
                }
                if (current == m_targetPositions[i])
                {
                    isTarget = true;
                    break;
                }
            }

            int iterator = 0;
            // Check if the current position is part of any unit's path
            for (const auto &path : paths)
            {
                if (std::find(path.begin(), path.end(), current) != path.end())
                {
                    isPath = true;
                    colorCode = 31 + (iterator %
                                      6); // Cycle through red, green, yellow, blue, magenta, cyan
                }
                iterator++;
            }

            // Print the appropriate symbol
            if (isStart)
            {
                std::cout << "S "; // Start
            }
            else if (isTarget)
            {
                std::cout << "T "; // Target
            }
            else if (isPath)
            {
                std::cout << "\033[" << colorCode << "m" << "P " << "\033[0m"; // Path
            }
            else if (m_map[x][y] == m_terrainKeys.at(Elevated))
            {
                std::cout << "# "; // Obstacle
            }
            else if (m_map[x][y] == m_terrainKeys.at(Reachable))
            {
                std::cout << ". "; // Free space
            }
        }
        std::cout << '\n';
    }
}