#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <string>

namespace fs = std::filesystem;

TEST(MissionPersistence, WriteAndReadMission) {
  auto tmp = fs::temp_directory_path() / "agv_test_missions";
  fs::create_directories(tmp);
  auto path = tmp / "missions.jsonl";

  // Write a mission
  {
    std::ofstream out(path);
    out << R"({"id":"m1","name":"Test patrol","created":1000.0,"waypoints":[{"x":1.0,"y":2.0,"theta":0.0,"action":"none","pause_sec":0}]})" << "\n";
  }

  // Read back
  std::ifstream in(path);
  std::string line;
  ASSERT_TRUE(std::getline(in, line));
  EXPECT_NE(line.find("\"id\":\"m1\""), std::string::npos);
  EXPECT_NE(line.find("\"Test patrol\""), std::string::npos);
  EXPECT_NE(line.find("\"x\":1"), std::string::npos);

  fs::remove_all(tmp);
}

TEST(MissionPersistence, AppendMultipleMissions) {
  auto tmp = fs::temp_directory_path() / "agv_test_missions2";
  fs::create_directories(tmp);
  auto path = tmp / "missions.jsonl";

  // Write two missions
  {
    std::ofstream out(path);
    out << R"({"id":"m1","name":"First"})" << "\n";
    out << R"({"id":"m2","name":"Second"})" << "\n";
  }

  // Count lines
  std::ifstream in(path);
  int count = 0;
  std::string line;
  while (std::getline(in, line)) {
    if (!line.empty()) count++;
  }
  EXPECT_EQ(count, 2);

  fs::remove_all(tmp);
}

TEST(MissionPersistence, FindMissionById) {
  auto tmp = fs::temp_directory_path() / "agv_test_missions3";
  fs::create_directories(tmp);
  auto path = tmp / "missions.jsonl";

  {
    std::ofstream out(path);
    out << R"({"id":"m1","name":"First"})" << "\n";
    out << R"({"id":"m2","name":"Second"})" << "\n";
    out << R"({"id":"m3","name":"Third"})" << "\n";
  }

  // Find m2
  std::ifstream in(path);
  std::string line;
  std::string found;
  while (std::getline(in, line)) {
    if (line.find("\"id\":\"m2\"") != std::string::npos) {
      found = line;
      break;
    }
  }
  EXPECT_FALSE(found.empty());
  EXPECT_NE(found.find("\"Second\""), std::string::npos);

  fs::remove_all(tmp);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
