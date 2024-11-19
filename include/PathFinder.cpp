// Local lib includes
#include "PathFinder.hpp"
#include "PathFinderConstants.hpp"

// External lib includes
#include <nlohmann/json.hpp> // Include nlohmann JSON library

// Standard Includes
#include <fstream>
#include <stdexcept>
#include <unordered_set>
#include <queue>
#include <iostream>
using json = nlohmann::json;
using namespace PathPlanner;

// Constructor that parses the config file
PathFinder::PathFinder()
{
    parseConfig(ConfigFilePath);
    parseMap(m_mapFilePath);
}

// Parse the configuration file
void PathFinder::parseConfig(const std::string &configFile)
{
    try
    {
        std::cout << "Parsing config file" << std::endl;
        std::ifstream file(configFile);
        if (!file.is_open())
        {
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
            throw std::runtime_error("Map file path not specified in config.");
        }

        // Required terrain keys
        const std::vector<std::string> requiredKeys = {Start, Target, Elevated, Reachable};

        // Validate all required keys are present
        for (const auto &key : requiredKeys)
        {
            if (m_terrainKeys.find(key) == m_terrainKeys.end())
            {
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

// Parse the map file
void PathFinder::parseMap(const std::string &mapFile)
{
    try
    {
        std::cout << "Parsing map data file" << std::endl;
        std::ifstream file(mapFile);
        bool startExists, targetExists = false;
        if (!file.is_open())
        {
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
            std::cerr << "Tilesets not found in JSON" << std::endl;
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
                        int index = i * width + j;
                        m_map[i][j] = int(data[index]);
                        if (m_map[i][j] == m_terrainKeys[Start])
                        {
                            m_startPosition = {i, j};
                            startExists = true;
                        }
                        else if (m_map[i][j] == m_terrainKeys[Target])
                        {
                            m_targetPosition = {i, j};
                            targetExists = true;
                        }
                    }
                }
            }
        }
        else
        {
            std::cerr << "'Data' key not found in first layer!" << std::endl;
        }

        if (!startExists || !targetExists)
        {
            std::cout << "Required points were not found in grid" << std::endl;
            std::cout << "Does start exist?: " << startExists << std::endl;
            std::cout << "Does target exist?: " << targetExists << std::endl;
            throw std::runtime_error("Missing start/target position in parseMap function.");
        }
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

bool PathFinder::isValidPosition(const Position &pos) const
{
    const int elevated = m_terrainKeys.at(Elevated);
    return pos.x >= 0 && pos.x < m_map.size() &&
           pos.y >= 0 && pos.y < m_map[0].size() &&
           m_map[pos.x][pos.y] != elevated;
}

int PathFinder::manhattanDistance(Position a, Position b) const
{
    return abs(a.x - b.x) + abs(a.y - b.y);
}

std::vector<PathFinder::Position> PathFinder::FindPath(const Position &start, const Position &target)
{
    std::cout << "Start position. X = " << start.x << " Y = " << start.y << std::endl;
    std::cout << "Target position. X = " << target.x << " Y = " << target.y << std::endl;

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> openList;
    std::unordered_map<Position, Node> allNodes; // Store nodes directly
    std::unordered_set<Position> closedList;

    // Initialize the start node
    allNodes[start] = Node(start, 0, manhattanDistance(start, target), nullptr);
    openList.push(allNodes[start]);

    while (!openList.empty())
    {
        Node current = openList.top();
        openList.pop();

        // Mark as visited
        closedList.insert(current.pos);

        // Check if target is reached
        if (current.pos == target)
        {
            std::vector<Position> path;
            for (const Node *node = &current; node != nullptr; node = node->parent)
            {
                path.push_back(node->pos);
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        // Explore neighbors
        std::vector<Position> neighbors = {
            {current.pos.x + 1, current.pos.y},
            {current.pos.x - 1, current.pos.y},
            {current.pos.x, current.pos.y + 1},
            {current.pos.x, current.pos.y - 1}};

        for (const auto &neighbor : neighbors)
        {
            // Skip invalid or already visited positions
            if (!isValidPosition(neighbor) || closedList.count(neighbor))
            {
                continue;
            }

            int gCost = current.gCost + 1;
            int hCost = manhattanDistance(neighbor, target);

            // Add new nodes or update existing ones if better path is found
            if (!allNodes.count(neighbor) || gCost < allNodes[neighbor].gCost)
            {
                allNodes[neighbor] = Node(neighbor, gCost, hCost, &allNodes[current.pos]);
                openList.push(allNodes[neighbor]);
            }
        }
    }

    // If no path is found
    std::cerr << "No path found." << std::endl;
    return {};
}
