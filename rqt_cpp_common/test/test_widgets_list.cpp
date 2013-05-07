#include <gtest/gtest.h>

#include "rqt_cpp_common/widgets_list.h"

TEST(TestWidgetsList, nullArg)
{
  rqt_cpp_common::WidgetsList wList;

}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
