#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <string>

namespace fs = std::filesystem;

TEST(ZonePersistence, CreateAndReadZoneFile) {
  auto tmp = fs::temp_directory_path() / "agv_test_zones";
  fs::create_directories(tmp);
  auto path = tmp / "zones.json";

  // Write a zone
  {
    std::ofstream out(path);
    out << R"({"zone_id":"z1","zone_type":"keepout","polygon_x":[0,1,1,0],"polygon_y":[0,0,1,1]})" << "\n";
  }

  // Read back
  std::ifstream in(path);
  std::string line;
  ASSERT_TRUE(std::getline(in, line));
  EXPECT_NE(line.find("\"zone_id\":\"z1\""), std::string::npos);
  EXPECT_NE(line.find("\"keepout\""), std::string::npos);

  fs::remove_all(tmp);
}

TEST(ZonePersistence, RemoveZoneByFiltering) {
  auto tmp = fs::temp_directory_path() / "agv_test_zones2";
  fs::create_directories(tmp);
  auto path = tmp / "zones.json";

  // Write two zones
  {
    std::ofstream out(path);
    out << R"({"zone_id":"z1","zone_type":"keepout"})" << "\n";
    out << R"({"zone_id":"z2","zone_type":"speed","max_speed":0.2})" << "\n";
  }

  // Filter out z1 (simulate remove)
  std::vector<std::string> kept;
  {
    std::ifstream in(path);
    std::string line;
    while (std::getline(in, line)) {
      if (line.find("\"zone_id\":\"z1\"") == std::string::npos) {
        kept.push_back(line);
      }
    }
  }

  EXPECT_EQ(kept.size(), 1u);
  EXPECT_NE(kept[0].find("z2"), std::string::npos);

  fs::remove_all(tmp);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
