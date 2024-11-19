#ifndef PATHFINDER_HPP
#define PATHFINDER_HPP

#include <vector>
#include <unordered_map>
#include <string>
#include <functional>

namespace PathPlanner
{
    class PathFinder
    {
    public:
        struct Position
        {
            int x, y;
            bool operator==(const Position &other) const
            {
                return x == other.x && y == other.y;
            }
        };

        struct Node
        {
            Position pos;
            int gCost, hCost;
            const Node *parent;

            // Default constructor
            Node()
                : pos(), gCost(0), hCost(0), parent(nullptr) {}

            // Parameterized constructor
            Node(const Position &position, int g, int h, const Node *parentNode)
                : pos(position), gCost(g), hCost(h), parent(parentNode) {}

            int fCost() const { return gCost + hCost; }

            bool operator>(const Node &other) const
            {
                return fCost() > other.fCost();
            }
        };
        // Constructors
        PathFinder();

        // Public methods
        std::vector<Position> FindPath(const Position &start, const Position &target);
        const std::vector<std::vector<int>> &GetMap() const { return m_map; }
        Position GetStartPosition() const { return m_startPosition; }
        Position GetTargetPosition() const { return m_targetPosition; }

    private:
        // Private members
        std::vector<std::vector<int>> m_map;
        Position m_startPosition;
        Position m_targetPosition;
        std::unordered_map<std::string, int> m_terrainKeys;
        std::string m_mapFilePath;

        // Private methods
        void parseConfig(const std::string &m_configFile);
        void parseMap(const std::string &mapFile);
        bool isValidPosition(const Position &pos) const;
        int manhattanDistance(Position a, Position b) const;
    };
}

// Specialization for hash<PathFinder::Position>
namespace std
{
    template <>
    struct hash<PathPlanner::PathFinder::Position>
    {
        size_t operator()(const PathPlanner::PathFinder::Position &p) const noexcept
        {
            return std::hash<int>()(p.x) ^ (std::hash<int>()(p.y) << 1);
        }
    };

}
#endif // PATHFINDER_HPP
