// Copyright 2021 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "gtest/gtest.h"
#include "mpc_lateral_controller/mpc_trajectory.hpp"
#include "mpc_lateral_controller/mpc_utils.hpp"

#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory_point.hpp"

#include <memory>
#include <vector>

namespace
{
namespace MPCUtils = autoware::motion::control::mpc_lateral_controller::MPCUtils;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;

TrajectoryPoint makePoint(const double x, const double y, const float vx = 0)
{
  TrajectoryPoint p;
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.longitudinal_velocity_mps = vx;
  return p;
}

/* cppcheck-suppress syntaxError */
TEST(TestMPC, calcDistance2d)
{
  Trajectory trajectory_msg;
  trajectory_msg.points.push_back(makePoint(0.0, 0.0));
  trajectory_msg.points.push_back(makePoint(1.0, 0.0));
  trajectory_msg.points.push_back(makePoint(1.0, 1.0));
  trajectory_msg.points.push_back(makePoint(4.0, 5.0));

  EXPECT_EQ(MPCUtils::calcDistance2d(trajectory_msg, 0, 1), 1.0);
  EXPECT_EQ(MPCUtils::calcDistance2d(trajectory_msg, 1, 2), 1.0);
  EXPECT_EQ(MPCUtils::calcDistance2d(trajectory_msg, 2, 3), 5.0);
}
}  // namespace
