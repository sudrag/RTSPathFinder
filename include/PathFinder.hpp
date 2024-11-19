#ifndef PATHFINDER_HPP
#define PATHFINDER_HPP

// Standard Includes
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

namespace PathPlanner
{
class PathFinder
{
  public:
    struct Position
    {
        int x = -1;
        int y = -1;

        // Default constructor
        Position() = default;

        // Parameterized constructor
        Position(int x, int y) : x(x), y(y) {}

        bool operator==(const Position &other) const { return x == other.x && y == other.y; }
    };

    struct Node
    {
        Position pos;
        int gCost, hCost;
        const Node *parent;

        // Default constructor
        Node() : pos(), gCost(0), hCost(0), parent(nullptr) {}

        // Parameterized constructor
        Node(const Position &position, int g, int h, const Node *parentNode)
            : pos(position), gCost(g), hCost(h), parent(parentNode)
        {
        }

        int fCost() const { return gCost + hCost; }

        bool operator>(const Node &other) const { return fCost() > other.fCost(); }
    };
    // Constructor
    PathFinder(const std::string &configFilePath);

    // Destructor
    ~PathFinder();

    // Public methods
    void FindPaths();
    const std::vector<std::vector<int>> &GetMap() const { return m_map; }
    Position GetStartPosition(int index) const;
    Position GetTargetPosition(int index) const;

  private:
    // Private members
    std::vector<std::vector<int>> m_map;
    std::vector<Position> m_startPositions;
    std::vector<Position> m_targetPositions;
    std::unordered_map<std::string, int> m_terrainKeys;
    std::string m_mapFilePath;

    // Private methods
    void parseConfig(const std::string &m_configFile);
    void parseMap(const std::string &mapFile);
    bool isValidPosition(const Position &pos) const;
    int manhattanDistance(Position a, Position b) const;
    bool hasCollision(const std::vector<Position> &positions, const Position &newPosition,
                      size_t currentIndex) const;
    void printMap(const std::vector<std::vector<Position>> &paths = {}) const;
    void validateMapPositions();
    std::vector<Position> getNeighborsforCurrentNode(Node currentNode);
    void printPaths(const std::vector<std::vector<Position>> &paths) const;

};
} // namespace PathPlanner

namespace std
{
// Custom hash function for PathPlanner::PathFinder::Position
// This function combines the hash values of x and y coordinates to produce a unique hash for each
// position. The combination uses XOR and bit-shifting to ensure that different (x, y) pairs yield
// different hash values.
template <> struct hash<PathPlanner::PathFinder::Position>
{
    size_t operator()(const PathPlanner::PathFinder::Position &p) const noexcept
    {
        return std::hash<int>()(p.x) ^ (std::hash<int>()(p.y) << 1);
    }
};

// Custom hash function for a vector of PathPlanner::PathFinder::Position
// This function iterates over each Position in the vector and accumulates the hash value
// using XOR and a golden ratio constant to reduce collision risk.
template <> struct hash<std::vector<PathPlanner::PathFinder::Position>>
{
    size_t
    operator()(const std::vector<PathPlanner::PathFinder::Position> &positions) const noexcept
    {
        size_t hash_value = 0;
        for (const auto &pos : positions)
        {
            // Calculate hash for each Position and combine it with the overall hash_value
            // Uses a golden ratio constant (0x9e3779b9) to help distribute the hash values
            // uniformly
            hash_value ^= (std::hash<int>()(pos.x) ^ (std::hash<int>()(pos.y) << 1)) + 0x9e3779b9 +
                          (hash_value << 6) + (hash_value >> 2);
        }
        return hash_value;
    }
};
} // namespace std

#endif // PATHFINDER_HPP
