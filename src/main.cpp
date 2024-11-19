// Local library includes
#include <PathFinder.hpp>

// Standard includes
#include <iostream>

int main()
{
    std::cout << "Path Finding algorithm for a Real-Time Stategy game" << std::endl;

    PathPlanner::PathFinder pathFinder;

    pathFinder.FindPaths();

    return 0;
}