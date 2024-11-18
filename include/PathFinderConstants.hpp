#ifndef PATHFINDER_CONSTANTS_HPP
#define PATHFINDER_CONSTANTS_HPP

#include <string>

namespace PathPlanner
{
    // Keys for JSON parsing
    inline const std::string TerrainKeys = "terrainKeys";
    inline const std::string MapFile = "mapFile";
    inline const std::string Start = "start";
    inline const std::string Target = "target";
    inline const std::string Elevated = "elevated";
    inline const std::string Reachable = "reachable";
    inline const std::string ConfigFilePath = "data/config.json";
    inline const std::string Tilesets = "tilesets";
    inline const std::string TileHeight = "tileheight";
    inline const std::string TileWidth = "tilewidth";
    inline const std::string Layers = "layers";
    inline const std::string Data = "data";

}

#endif // PATHFINDER_CONSTANTS_HPP
