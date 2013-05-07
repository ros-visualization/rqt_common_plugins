#include <gtest/gtest.h>

#include <marble/GeoDataCoordinates.h>

#include "rqt_marble/bridge_ros_marble.h"
#include "rqt_marble/marble_plugin.h"

TEST(TestBridgeRosMarble, nullArg)
{
  rqt_marble::BridgeRosMarble bridge;

  //Create Marble::Route instance and its values.
  Marble::Route route;
  Marble::GeoDataCoordinates *position1 = new Marble::GeoDataCoordinates(
      12.1, 14.3, 0.0, Marble::GeoDataCoordinates::Degree);
  rqt_cpp_common::RouteGps r = bridge.publishRouteInGps(route);
  EXPECT_EQ(1, sizeof(r.routes));
}

//TEST(TestFindGpsTopics, dummyNavSatFix)
//{
//  rqt_marble::BridgeRosMarble bridge;
//  Marble::Route route;
//  EXPECT_EQ(NULL, bridge.publishRouteInGps(route));
//}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
