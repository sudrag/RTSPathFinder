// Local library includes
#include <PathFinder.hpp>

// Standard includes
#include <iostream>

int main() {
    std::cout << "Path Finding algorithm for a Real-Time Stategy game" << std::endl;
    
 try {
        PathPlanner::PathFinder pathFinder;

        auto path = pathFinder.FindPath(pathFinder.GetStartPosition(), pathFinder.GetTargetPosition());

        if (path.empty()) {
            std::cout << "No path found!" << std::endl;
        } else {
            std::cout << "Path found:" << std::endl;
            for (const auto& pos : path) {
                std::cout << "(" << pos.x << ", " << pos.y << ") ";
            }
            std::cout << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }

    return 0;
}