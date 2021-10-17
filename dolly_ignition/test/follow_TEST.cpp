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

#include <ignition/common/Console.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/World.hh>
#include <ignition/gazebo/Server.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/TestFixture.hh>

#include <memory>
#include <vector>

//////////////////////////////////////////////////
//
TEST(DollyTests, Follow)
{
  // Maximum verbosity helps with debugging
  ignition::common::Console::SetVerbosity(4);

  // Instantiate test fixture. It starts a simulation server and provides
  // hooks that we'll use to inspect the running simulation.
  ignition::gazebo::TestFixture fixture("empty.sdf");

  int iterations{0};
  ignition::gazebo::Entity dollyEntity;
  std::vector<ignition::math::Pose3d> dollyPoses;

  fixture.
  // Use configure callback to get values at startup
  OnConfigure(
    [&dollyEntity, &dollyPoses](const ignition::gazebo::Entity & _worldEntity,
    const std::shared_ptr<const sdf::Element> & /*_sdf*/,
    ignition::gazebo::EntityComponentManager & _ecm,
    ignition::gazebo::EventManager & /*_eventMgr*/)
    {
      // Get world
      ignition::gazebo::World world(_worldEntity);

      // Get dolly entity once it's spawned
//      dollyEntity = world.ModelByName(_ecm, "dolly");
//      EXPECT_NE(ignition::gazebo::kNullEntity, dollyEntity);
    }).
  // Use post-update callback to get values at the end of every iteration
  OnPostUpdate(
    [&iterations, &dollyEntity, &dollyPoses](
      const ignition::gazebo::UpdateInfo & _info,
      const ignition::gazebo::EntityComponentManager & _ecm)
    {
      // Inspect all model poses
      dollyPoses.push_back(ignition::gazebo::worldPose(dollyEntity, _ecm));

//      EXPECT_DOUBLE_EQ(0.0, pose.Pos().X());
//      EXPECT_DOUBLE_EQ(0.0, pose.Pos().Y());

      iterations++;
    }).
  // The moment we finalize, the configure callback is called
  Finalize();

  // Setup simulation server, this will call the post-update callbacks.
  // It also calls pre-update and update callbacks if those are being used.
  fixture.Server()->Run(true, 1000, false);

  // Verify that the post update function was called 1000 times
  EXPECT_EQ(1000, iterations);
}
