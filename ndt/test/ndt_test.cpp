#include "ndt/ndt.hpp"

#include "gtest/gtest.h"

class NDTLocalizationTest : public localization::NDTLocalization {};

TEST(ndt_test, get_initial_pose) {
  double eps = 0.0001f;
  ASSERT_NEAR(0, 0, eps);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
