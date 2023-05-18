/*
 * Copyright (C) 2023 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <gz/common/Console.hh>
#include <gz/plugin/Loader.hh>

#include <gz/physics/FindFeatures.hh>
#include <gz/physics/ForwardStep.hh>
#include <gz/physics/FrameSemantics.hh>
#include <gz/physics/GetBoundingBox.hh>
#include <gz/physics/RequestEngine.hh>
#include <gz/physics/World.hh>

#include <gz/physics/sdf/ConstructWorld.hh>


#include <sdf/Root.hh>
#include <sdf/World.hh>

#include "test/Utils.hh"
#include "TestUtilities.hh"

struct TestFeatureList : gz::physics::FeatureList<
    gz::physics::CollisionDetector,
    gz::physics::Gravity,
    gz::physics::LinkFrameSemantics,
    gz::physics::Solver,
    gz::physics::ForwardStep,
    gz::physics::sdf::ConstructSdfWorld,
    gz::physics::GetEntities
> { };

using TestEnginePtr = gz::physics::Engine3dPtr<TestFeatureList>;
using TestWorldPtr = gz::physics::World3dPtr<TestFeatureList>;
using AssertVectorApprox = gz::physics::test::AssertVectorApprox;

TestWorldPtr LoadWorld(
    const TestEnginePtr &_engine,
    const std::string &_sdfFile)
{
  sdf::Root root;
  const sdf::Errors errors = root.Load(_sdfFile);
  EXPECT_TRUE(errors.empty());
  const sdf::World *sdfWorld = root.WorldByIndex(0);
  return _engine->ConstructWorld(*sdfWorld);
}

int main(int argc, const char* argv[])
{
  gz::plugin::Loader loader;
  PrimeTheLoader(loader);

  gz::plugin::PluginPtr dartsim =
      loader.Instantiate("gz::physics::dartsim::Plugin");

  auto engine = gz::physics::RequestEngine3d<TestFeatureList>::From(dartsim);

  auto world = LoadWorld(engine,
      gz::common::joinPaths(TEST_WORLD_DIR, "empty.sdf"));

  world->SetCollisionDetector("dart");
  world->GetCollisionDetector();

  for (const std::string &library
       : physics::test::g_PhysicsPluginLibraries)
  {
    if (!library.empty())
    {
      bool result = loader.ForgetLibrary(library);
      std::cerr << "Forgetting library: " << library
                << " [" << (result ? "Pass" : "Fail") << "]\n";
    }
  }
}
