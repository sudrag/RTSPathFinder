#include "../include/PathFinder.hpp"

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include <fstream>

using namespace PathPlanner;
using Position = PathPlanner::PathFinder::Position;

// Utility function to write JSON content to a file
void writeJsonToFile(const std::string &filePath, const nlohmann::json &jsonContent)
{
    std::ofstream file(filePath);
    if (file.is_open())
    {
        file << jsonContent.dump(4); // Pretty print with an indentation of 4 spaces
        file.close();
    }
    else
    {
        throw std::ios_base::failure("Failed to open file for writing");
    }
}

// Test fixture for PathFinder tests
class PathFinderTest : public ::testing::Test
{
  protected:
    void SetUp() override
    {
        // Create a sample config file
        nlohmann::json config = {
            {"mapFile", "test_map.json"},
            {"terrainKeys", {{"start", 0}, {"target", 8}, {"elevated", 3}, {"reachable", -1}}}};
        writeJsonToFile("test_config.json", config);

        // Create a sample map file
        nlohmann::json mapData;
        mapData["layers"] = {
            {{"name", "world"},
             {"tileset", "MapEditor Tileset_woodland.png"},
             {"data", {0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 8}}}};
        mapData["tilesets"] = {{{"name", "MapEditor Tileset_woodland.png"},
                                {"image", "MapEditor Tileset_woodland.png"},
                                {"imagewidth", 512},
                                {"imageheight", 512},
                                {"tilewidth", 4},
                                {"tileheight", 4}}};
        mapData["canvas"] = {{"width", 1024}, {"height", 1024}};
        writeJsonToFile("test_map.json", mapData);
    }

    void TearDown() override
    {
        // Clean up files
        remove("test_config.json");
        remove("test_map.json");
    }
};

// Test parseConfig, parseMap and Constructor
TEST_F(PathFinderTest, SimpleConstructor)
{
    EXPECT_NO_THROW(PathFinder pathFinder("test_config.json"));
}

// Test parseConfig, parseMap and Constructor
TEST_F(PathFinderTest, MapParser)
{
    PathFinder pathFinder("test_config.json");
    Position start = pathFinder.GetStartPosition(0);
    Position target = pathFinder.GetTargetPosition(0);
    Position expected_start{0, 0};
    Position expected_target{3, 3};
    auto map = pathFinder.GetMap();
    int num_rows = map.size();
    int num_columns = map[0].size();
    // Expect map to be of expected dimensions
    EXPECT_EQ(num_rows, 4);
    EXPECT_EQ(num_columns, 4);
    EXPECT_EQ(expected_start, start);
    EXPECT_EQ(expected_target, target);
}

// Test parseConfig, parseMap and Constructor for multiple units
TEST_F(PathFinderTest, MapParserMultipleUnits)
{
    nlohmann::json mapData;
    mapData["layers"] = {{{"name", "world"},
                          {"tileset", "MapEditor Tileset_woodland.png"},
                          {"data", {0, -1, 0, -1, -1, -1, -1, -1, -1, 8, -1, -1, 3, -1, 8, 3}}}};
    mapData["tilesets"] = {{{"name", "MapEditor Tileset_woodland.png"},
                            {"image", "MapEditor Tileset_woodland.png"},
                            {"imagewidth", 512},
                            {"imageheight", 512},
                            {"tilewidth", 4},
                            {"tileheight", 4}}};
    mapData["canvas"] = {{"width", 1024}, {"height", 1024}};
    writeJsonToFile("test_map.json", mapData);

    PathFinder pathFinder("test_config.json");
    Position start_unit0 = pathFinder.GetStartPosition(0);
    Position target_unit0 = pathFinder.GetTargetPosition(0);
    Position expected_start_unit0 = {0, 0};
    Position expected_target_unit0 = {2, 1};
    Position start_unit1 = pathFinder.GetStartPosition(1);
    Position target_unit1 = pathFinder.GetTargetPosition(1);
    Position expected_start_unit1 = {0, 2};
    Position expected_target_unit1 = {3, 2};
    auto map = pathFinder.GetMap();
    int num_rows = map.size();
    int num_columns = map[0].size();
    // Expect map to be of expected dimensions
    EXPECT_EQ(num_rows, 4);
    EXPECT_EQ(num_columns, 4);
    // Expect the targets and start positions for the units to match
    EXPECT_EQ(expected_start_unit0, start_unit0);
    EXPECT_EQ(expected_target_unit0, target_unit0);
    EXPECT_EQ(expected_start_unit1, start_unit1);
    EXPECT_EQ(expected_target_unit1, target_unit1);
}

// Test parseConfig, parseMap and Constructor for map with decimals
TEST_F(PathFinderTest, MapParserWithDecimals)
{
    nlohmann::json mapData;
    mapData["layers"] = {
        {{"name", "world"},
         {"tileset", "MapEditor Tileset_woodland.png"},
         {"data", {0.4, -1.6, 0.3, -1.3, -1.5, -1, -1, -1, -1, 3.2, -1, -1, 8.4, -1, -1, 8.2}}}};
    mapData["tilesets"] = {{{"name", "MapEditor Tileset_woodland.png"},
                            {"image", "MapEditor Tileset_woodland.png"},
                            {"imagewidth", 512},
                            {"imageheight", 512},
                            {"tilewidth", 4},
                            {"tileheight", 4}}};
    mapData["canvas"] = {{"width", 1024}, {"height", 1024}};
    writeJsonToFile("test_map.json", mapData);

    PathFinder pathFinder("test_config.json");
    Position start_unit0 = pathFinder.GetStartPosition(0);
    Position target_unit0 = pathFinder.GetTargetPosition(0);
    Position expected_start_unit0 = {0, 0};
    Position expected_target_unit0 = {3, 0};
    Position start_unit1 = pathFinder.GetStartPosition(1);
    Position target_unit1 = pathFinder.GetTargetPosition(1);
    Position expected_start_unit1 = {0, 2};
    Position expected_target_unit1 = {3, 3};
    auto map = pathFinder.GetMap();
    int num_rows = map.size();
    int num_columns = map[0].size();
    // Expect map to be of expected dimensions
    EXPECT_EQ(num_rows, 4);
    EXPECT_EQ(num_columns, 4);
    // Expect target and start positions to match the given integer values
    EXPECT_EQ(expected_start_unit0, start_unit0);
    EXPECT_EQ(expected_target_unit0, target_unit0);
    EXPECT_EQ(expected_start_unit1, start_unit1);
    EXPECT_EQ(expected_target_unit1, target_unit1);
}

// Test parseConfig, parseMap with multiple unit starts but only one target
TEST_F(PathFinderTest, TargetDuplication)
{
    nlohmann::json mapData;
    mapData["layers"] = {{{"name", "world"},
                          {"tileset", "MapEditor Tileset_woodland.png"},
                          {"data", {0, -1, 0, -1, 0, -1, -1, -1, -1, 3, -1, -1, 3, -1, -1, 8}}}};
    mapData["tilesets"] = {{{"name", "MapEditor Tileset_woodland.png"},
                            {"image", "MapEditor Tileset_woodland.png"},
                            {"imagewidth", 512},
                            {"imageheight", 512},
                            {"tilewidth", 4},
                            {"tileheight", 4}}};
    mapData["canvas"] = {{"width", 1024}, {"height", 1024}};
    writeJsonToFile("test_map.json", mapData);

    PathFinder pathFinder("test_config.json");
    Position start_unit0 = pathFinder.GetStartPosition(0);
    Position target_unit0 = pathFinder.GetTargetPosition(0);
    Position expected_start_unit0 = {0, 0};
    Position expected_target_unit0 = {3, 3};
    Position start_unit1 = pathFinder.GetStartPosition(1);
    Position target_unit1 = pathFinder.GetTargetPosition(1);
    Position expected_start_unit1 = {0, 2};
    Position start_unit2 = pathFinder.GetStartPosition(2);
    Position target_unit2 = pathFinder.GetTargetPosition(2);
    Position expected_start_unit2 = {1, 0};
    auto map = pathFinder.GetMap();
    int num_rows = map.size();
    int num_columns = map[0].size();
    // Expect map to be of expected dimensions
    EXPECT_EQ(num_rows, 4);
    EXPECT_EQ(num_columns, 4);
    // Expect start units to match the co-ordinates. Expect target units to match the duplicated
    // target
    EXPECT_EQ(expected_start_unit0, start_unit0);
    EXPECT_EQ(expected_target_unit0, target_unit0);
    EXPECT_EQ(expected_start_unit1, start_unit1);
    EXPECT_EQ(expected_target_unit0, target_unit1);
    EXPECT_EQ(expected_start_unit2, start_unit2);
    EXPECT_EQ(expected_target_unit0, target_unit2);
}

// Test FindPaths method
TEST_F(PathFinderTest, FindPathsNoObstacles)
{
    // Happy path test to find the path with no obstacles
    PathFinder pathFinder("test_config.json");
    EXPECT_NO_THROW(pathFinder.FindPaths());
}

// // Test with only obstacles in the map
TEST_F(PathFinderTest, FindPathsOnlyObstacles)
{
    nlohmann::json mapData;
    mapData["layers"] = {{{"name", "world"},
                          {"tileset", "MapEditor Tileset_woodland.png"},
                          {"data", {0, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 8}}}};
    mapData["tilesets"] = {{{"name", "MapEditor Tileset_woodland.png"},
                            {"image", "MapEditor Tileset_woodland.png"},
                            {"imagewidth", 512},
                            {"imageheight", 512},
                            {"tilewidth", 4},
                            {"tileheight", 4}}};
    mapData["canvas"] = {{"width", 1024}, {"height", 1024}};
    writeJsonToFile("test_map.json", mapData);

    PathFinder pathFinder("test_config.json");
    // No paths should be found since the map is fully blocked
    EXPECT_NO_THROW(pathFinder.FindPaths());
}

// Test parseConfig and parseMap for modified config file
TEST_F(PathFinderTest, MapParserCustomConfig)
{
    // Create a modified config file with different terrain values for the same keys
    nlohmann::json config = {
        {"mapFile", "test_map.json"},
        {"terrainKeys", {{"start", 4}, {"target", 6}, {"elevated", 0}, {"reachable", 8}}}};
    writeJsonToFile("test_config.json", config);

    // Create a modified map to match the custom keys above
    nlohmann::json mapData;
    mapData["layers"] = {{{"name", "world"},
                          {"tileset", "MapEditor Tileset_woodland.png"},
                          {"data", {4, 8, 0, 0, 8, 8, 8, 8, 8, 8, 8, 8, 0, 8, 8, 6}}}};
    mapData["tilesets"] = {{{"name", "MapEditor Tileset_woodland.png"},
                            {"image", "MapEditor Tileset_woodland.png"},
                            {"imagewidth", 512},
                            {"imageheight", 512},
                            {"tilewidth", 4},
                            {"tileheight", 4}}};
    mapData["canvas"] = {{"width", 1024}, {"height", 1024}};
    writeJsonToFile("test_map.json", mapData);

    PathFinder pathFinder("test_config.json");
    Position start_unit0 = pathFinder.GetStartPosition(0);
    Position target_unit0 = pathFinder.GetTargetPosition(0);
    Position expected_start_unit0{0, 0};
    Position expected_target_unit0{3, 3};
    auto map = pathFinder.GetMap();
    int num_rows = map.size();
    int num_columns = map[0].size();
    // Expect map to be of expected dimensions
    EXPECT_EQ(num_rows, 4);
    EXPECT_EQ(num_columns, 4);
    // Expect parser to have worked and found the appropriate start and target position
    EXPECT_EQ(expected_start_unit0, start_unit0);
    EXPECT_EQ(expected_target_unit0, target_unit0);
}

// Test parseConfig and parseMap for incorrect config file
TEST_F(PathFinderTest, BadConfigFileWithMissingTerrain)
{
    // Create a modified config file with missing terrain value for elevated
    nlohmann::json config = {
        {"mapFile", "test_map.json"},
        {"terrainKeys", {{"start", 4}, {"target", 6}, {"reachable", 8}}}};
    writeJsonToFile("test_config.json", config);

    EXPECT_ANY_THROW(PathFinder pathFinder("test_config.json"));
}

// Test parseConfig and parseMap for incorrect config file with bad path
TEST_F(PathFinderTest, BadConfigFileWithIncorrectFilePath)
{
    // Create a modified config file with bad map file path
    nlohmann::json config = {
        {"mapFile", "badMapPath.json"},
        {"terrainKeys", {{"start", 4}, {"target", 6}, {"elevated", 0}, {"reachable", 8}}}};
    writeJsonToFile("test_config.json", config);

    EXPECT_ANY_THROW(PathFinder pathFinder("test_config.json"));
}

// Test parseConfig and parseMap for incorrect map file with missing data
TEST_F(PathFinderTest, BadMapFileWithMissingData)
{
    // Create a modified config file with different terrain values for the same keys
    nlohmann::json config = {
        {"mapFile", "test_map.json"},
        {"terrainKeys", {{"start", 4}, {"target", 6}, {"elevated", 0}, {"reachable", 8}}}};
    writeJsonToFile("test_config.json", config);

    // Create a bad map file with missing data field
    nlohmann::json mapData;
    mapData["layers"] = {{{"name", "world"},
                          {"tileset", "MapEditor Tileset_woodland.png"}}};
    mapData["tilesets"] = {{{"name", "MapEditor Tileset_woodland.png"},
                            {"image", "MapEditor Tileset_woodland.png"},
                            {"imagewidth", 512},
                            {"imageheight", 512},
                            {"tilewidth", 4},
                            {"tileheight", 4}}};
    mapData["canvas"] = {{"width", 1024}, {"height", 1024}};
    writeJsonToFile("test_map.json", mapData);


    EXPECT_ANY_THROW(PathFinder pathFinder("test_config.json"));
}

// Test parseConfig and parseMap for incorrect map file with missing dimensions
TEST_F(PathFinderTest, BadMapFileWithMissingDimension)
{
    // Create a modified config file with different terrain values for the same keys
    nlohmann::json config = {
        {"mapFile", "test_map.json"},
        {"terrainKeys", {{"start", 4}, {"target", 6}, {"elevated", 0}, {"reachable", 8}}}};
    writeJsonToFile("test_config.json", config);

    // Create a bad map file with missing tilesets
    nlohmann::json mapData;
    mapData["layers"] = {{{"name", "world"},
                          {"tileset", "MapEditor Tileset_woodland.png"}}};
    mapData["canvas"] = {{"width", 1024}, {"height", 1024}};
    writeJsonToFile("test_map.json", mapData);


    EXPECT_ANY_THROW(PathFinder pathFinder("test_config.json"));
}


int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
