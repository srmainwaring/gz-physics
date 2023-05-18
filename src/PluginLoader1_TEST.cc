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

#include <iostream>

#include <gz/plugin/Loader.hh>

#include "TestUtilities.hh"

int main(int argc, const char* argv[])
{
  gz::plugin::Loader loader;
  PrimeTheLoader(loader);

  for (const std::string &library : physics::test::g_PhysicsPluginLibraries)
  {
    if (!library.empty())
    {
      bool result = loader.ForgetLibrary(library);
      std::cerr << "Forgetting library: " << library
                << " [" << (result ? "Pass" : "Fail") << "]\n";
    }
  }
}
