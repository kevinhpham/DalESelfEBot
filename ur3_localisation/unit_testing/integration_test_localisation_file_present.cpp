#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include <fstream>

TEST(LocalisationFileTest, FileExistsAndIsReadable) {
    std::string file_path = "/home/jarred/git/DalESelfEBot/ur3_localisation/config/hardware_params.yaml";

    std::ifstream file(file_path);
    ASSERT_TRUE(file.good()) << "File not found at path: " << file_path;
    file.close();

    // Try loading the YAML
    YAML::Node config = YAML::LoadFile(file_path);
    ASSERT_TRUE(config["corner_positions"]) << "'corner_positions' key missing.";
    ASSERT_TRUE(config["corner_positions"].IsSequence()) << "'corner_positions' must be a sequence.";
    ASSERT_EQ(config["corner_positions"].size(), 4) << "Expected 4 corner positions.";
}
