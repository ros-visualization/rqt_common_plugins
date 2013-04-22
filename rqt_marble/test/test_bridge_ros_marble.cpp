#include <gtest/gtest.h>

#include "rqt_marble/bridge_ros_marble.h"
#include "rqt_marble/marble_plugin.h"

class TestBridgeRosMarble : public ::testing::Test
{

  TEST(TestBridgeRosMarble, nullArg)
  {
    BridgeRosMarble bridge;
    EXPECT_EQ(NULL, bridge.publishRouteInGps(NULL));
  }

  TEST(TestFindGpsTopics, dummyNavSatFix)
  {
    BridgeRosMarble bridge;
    EXPECT_EQ(NULL, bridge.publishRouteInGps(NULL));
  }
};

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
