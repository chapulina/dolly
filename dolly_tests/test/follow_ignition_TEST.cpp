// Copyright 2021 Louise Poubel.
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

#include <gtest/gtest.h>

#include <ignition/msgs/entity_factory.pb.h>

#include <ignition/common/Console.hh>
#include <ignition/gazebo/World.hh>
#include <ignition/gazebo/Server.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/TestFixture.hh>
#include <ignition/transport/Node.hh>

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "constants.hh"

using namespace std::chrono_literals;

//////////////////////////////////////////////////
TEST(DollyTests, Follow)
{
  // Maximum verbosity helps with debugging
  ignition::common::Console::SetVerbosity(4);

  // Instantiate test fixture. It starts a simulation server and provides
  // hooks that we'll use to inspect the running simulation.
  ignition::gazebo::ServerConfig config;
  config.SetSdfFile(ignition::common::joinPaths(
      std::string(PROJECT_SOURCE_PATH), "worlds", "empty.sdf"));
  config.SetHeadlessRendering(true);

  ignition::gazebo::TestFixture fixture(config);

  // Variables that will be populated during the simulation
  int iterations{0};
  ignition::gazebo::World world;
  ignition::gazebo::Entity dollyEntity{ignition::gazebo::kNullEntity};
  std::vector<ignition::math::Pose3d> dollyPoses;
  ignition::gazebo::Entity targetEntity{ignition::gazebo::kNullEntity};

  fixture.
  // Use configure callback to get values at startup
  OnConfigure(
    [&](const ignition::gazebo::Entity & _worldEntity,
    const std::shared_ptr<const sdf::Element> &,
    ignition::gazebo::EntityComponentManager & _ecm,
    ignition::gazebo::EventManager &)
    {
      // Get world
      world = ignition::gazebo::World(_worldEntity);
    }).
  // Use post-update callback to get values at the end of every iteration
  OnPostUpdate(
    [&](
      const ignition::gazebo::UpdateInfo &,
      const ignition::gazebo::EntityComponentManager & _ecm)
    {
      iterations++;

      // Get dolly entity once it's spawned
      dollyEntity = world.ModelByName(_ecm, "dolly");
      if (ignition::gazebo::kNullEntity == dollyEntity) {
        return;
      }

      EXPECT_NE(ignition::gazebo::kNullEntity, dollyEntity);

      // Inspect all model poses
      dollyPoses.push_back(ignition::gazebo::worldPose(dollyEntity, _ecm));

      // Get target entity once it's spawned
      targetEntity = world.ModelByName(_ecm, "target");
    }).
  // The moment we finalize, the configure callback is called
  Finalize();

  // Run simulation server, this will call the post-update callbacks.
  int sleep = 0;
  int maxSleep = 30;
  for (; sleep <= maxSleep && ignition::gazebo::kNullEntity == dollyEntity;
    ++sleep)
  {
    std::this_thread::sleep_for(100ms);
    fixture.Server()->Run(true /*blocking*/, 100, false /*paused*/);
  }

  EXPECT_LT(sleep, maxSleep);
  EXPECT_EQ(100 * sleep, iterations);
  EXPECT_NE(ignition::gazebo::kNullEntity, dollyEntity);
  EXPECT_LT(0u, dollyPoses.size());

  // Check that Dolly didn't move, because there's nothing to follow
  for (auto i = 0; i < dollyPoses.size(); ++i) {
    const auto & pose = dollyPoses[i];
    EXPECT_NEAR(0.0, pose.Pos().X(), 1e-3) << i;
    EXPECT_NEAR(0.0, pose.Pos().Y(), 1e-3) << i;
    EXPECT_NEAR(0.22, pose.Pos().Z(), 1e-2) << i;
    EXPECT_NEAR(0.0, pose.Rot().Roll(), 1e-3) << i;
    EXPECT_NEAR(0.0, pose.Rot().Pitch(), 1e-3) << i;
    EXPECT_NEAR(0.0, pose.Rot().Yaw(), 1e-3) << i;
  }

  // Spawn an object in front of Dolly, to the right
  const auto modelStr = std::string("<?xml version=\"1.0\" ?>") +
    "<sdf version='1.6'>" +
    "<model name='target'>" +
    "<link name='link'>" +
    "<visual name='visual'>" +
    "<geometry><sphere><radius>1.0</radius></sphere></geometry>" +
    "</visual>" +
    "<collision name='visual'>" +
    "<geometry><sphere><radius>1.0</radius></sphere></geometry>" +
    "</collision>" +
    "</link>" +
    "</model>" +
    "</sdf>";

  ignition::msgs::EntityFactory req;
  req.set_sdf(modelStr);

  auto pose = req.mutable_pose();
  auto pos = pose->mutable_position();
  pos->set_x(5);
  pos->set_y(-3);

  ignition::msgs::Boolean res;
  bool result;
  unsigned int timeout = 2000;
  std::string service{"/world/empty/create"};

  ignition::transport::Node node;
  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(res.data());

  // Run simulation until target is spawned
  iterations = 0;
  sleep = 0;
  for (; sleep <= maxSleep && ignition::gazebo::kNullEntity == targetEntity;
    ++sleep)
  {
    std::this_thread::sleep_for(100ms);
    fixture.Server()->Run(true, 10, false);
  }

  EXPECT_LT(sleep, maxSleep);
  EXPECT_EQ(10 * sleep, iterations);
  EXPECT_NE(ignition::gazebo::kNullEntity, targetEntity);
  EXPECT_LT(0u, dollyPoses.size());

  // Dolly hasn't moved yet
  {
    const auto & pose = dollyPoses.back();
    EXPECT_NEAR(0.0, pose.Pos().X(), 1e-3);
    EXPECT_NEAR(0.0, pose.Pos().Y(), 1e-3);
    EXPECT_NEAR(0.22, pose.Pos().Z(), 1e-2);
    EXPECT_NEAR(0.0, pose.Rot().Roll(), 1e-3);
    EXPECT_NEAR(0.0, pose.Rot().Pitch(), 1e-3);
    EXPECT_NEAR(0.0, pose.Rot().Yaw(), 2e-3);
  }

  // Run simulation and check that Dolly moves towards target
  iterations = 0;
  dollyPoses.clear();
  sleep = 0;
  for (; sleep <= maxSleep && dollyPoses.back().Pos().X() < 1.0; ++sleep) {
    std::this_thread::sleep_for(100ms);
    fixture.Server()->Run(true, 1000, false);
  }

  EXPECT_LT(sleep, maxSleep);
  EXPECT_EQ(1000 * sleep, iterations);
  EXPECT_NE(ignition::gazebo::kNullEntity, targetEntity);
  EXPECT_LT(4000u, dollyPoses.size());

  ignwarn << "Recorded [" << dollyPoses.size() << "] poses" << std::endl;

  for (auto i = 2000; i < dollyPoses.size(); i = i + 100) {
    if (i == 2000) {
      continue;
    }

    const auto & prevPose = dollyPoses[i - 100];
    const auto & pose = dollyPoses[i];

    // Going forward
    EXPECT_LT(prevPose.Pos().X(), pose.Pos().X()) << i;

    // Going right
    EXPECT_GT(prevPose.Pos().Y(), pose.Pos().Y()) << i;

    // Turning right
    EXPECT_GT(prevPose.Rot().Yaw(), pose.Rot().Yaw()) << i;

    // Not flying, rolling or pitching
    EXPECT_NEAR(prevPose.Pos().Z(), pose.Pos().Z(), 1e-3) << i;
    EXPECT_NEAR(prevPose.Rot().Roll(), pose.Rot().Roll(), 1e-3) << i;
    EXPECT_NEAR(prevPose.Rot().Pitch(), pose.Rot().Pitch(), 1e-3) << i;
  }

  {
    const auto & pose = dollyPoses.back();
    EXPECT_LT(1.0, pose.Pos().X());
    EXPECT_GT(-0.2, pose.Pos().Y());
    EXPECT_NEAR(0.22, pose.Pos().Z(), 1e-2);
    EXPECT_NEAR(0.0, pose.Rot().Roll(), 1e-3);
    EXPECT_NEAR(0.0, pose.Rot().Pitch(), 1e-3);
    EXPECT_GT(-0.5, pose.Rot().Yaw());
  }
}
