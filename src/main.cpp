// Local library includes
#include <PathFinder.hpp>

// Standard includes
#include <iostream>

int main()
{
    std::cout << "Path Finding algorithm for a Real-Time Stategy game" << std::endl;

    const std::string filePath = "data/config.json";
    PathPlanner::PathFinder pathFinder(filePath);

    pathFinder.FindPaths();

    return 0;
}