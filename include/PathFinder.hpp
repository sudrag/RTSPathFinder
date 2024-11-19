#ifndef PATHFINDER_HPP
#define PATHFINDER_HPP

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
};
} // namespace PathPlanner

// Specialization for hash<PathFinder::Position>
namespace std
{
template <> struct hash<PathPlanner::PathFinder::Position>
{
    size_t operator()(const PathPlanner::PathFinder::Position &p) const noexcept
    {
        return std::hash<int>()(p.x) ^ (std::hash<int>()(p.y) << 1);
    }
};

} // namespace std

namespace std
{
template <> struct hash<std::vector<PathPlanner::PathFinder::Position>>
{
    size_t
    operator()(const std::vector<PathPlanner::PathFinder::Position> &positions) const noexcept
    {
        size_t hash_value = 0;
        for (const auto &pos : positions)
        {
            hash_value ^= (std::hash<int>()(pos.x) ^ (std::hash<int>()(pos.y) << 1)) + 0x9e3779b9 +
                          (hash_value << 6) + (hash_value >> 2);
        }
        return hash_value;
    }
};
} // namespace std

#endif // PATHFINDER_HPP
