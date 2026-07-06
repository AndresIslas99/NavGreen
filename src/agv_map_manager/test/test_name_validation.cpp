#include <gtest/gtest.h>

#include <string>

#include "agv_map_manager/name_validation.hpp"

using agv_map_manager::is_safe_name;

TEST(NameValidation, AcceptsTypicalMapNames) {
  EXPECT_TRUE(is_safe_name("corridor_v1"));
  EXPECT_TRUE(is_safe_name("default_empty"));
  EXPECT_TRUE(is_safe_name("Map-2026-07-06"));
  EXPECT_TRUE(is_safe_name("z1"));
}

TEST(NameValidation, RejectsEmptyAndOverlong) {
  EXPECT_FALSE(is_safe_name(""));
  EXPECT_TRUE(is_safe_name(std::string(64, 'a')));
  EXPECT_FALSE(is_safe_name(std::string(65, 'a')));
}

TEST(NameValidation, RejectsPathTraversal) {
  EXPECT_FALSE(is_safe_name("../etc/passwd"));
  EXPECT_FALSE(is_safe_name("a/b"));
  EXPECT_FALSE(is_safe_name(".."));
  EXPECT_FALSE(is_safe_name("map.yaml"));
}

TEST(NameValidation, RejectsShellInjection) {
  // Neither '/' nor ".." appears here — the old check let this through
  // and the single quote escaped the popen command quoting.
  EXPECT_FALSE(is_safe_name("x'; rm -rf $HOME; echo '"));
  EXPECT_FALSE(is_safe_name("a b"));
  EXPECT_FALSE(is_safe_name("a$(reboot)"));
  EXPECT_FALSE(is_safe_name("a`id`"));
  EXPECT_FALSE(is_safe_name("a;b"));
}

TEST(NameValidation, RejectsJsonBreakingQuotes) {
  EXPECT_FALSE(is_safe_name("z\"1"));
  EXPECT_FALSE(is_safe_name("z\\1"));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
