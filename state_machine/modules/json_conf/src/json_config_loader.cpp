// Copyright 2023 Klepsydra Technologies AG
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cereal/archives/json.hpp>
#include <fstream>

#include <klepsydra/state_machine/json_config_loader.h>
#include <klepsydra/state_machine/json_to_state_machine.h>

namespace kpsr {
namespace fsm {
ConfigStateMachine JsonConfigLoader::loadConfig(const std::string &specPath)
{
    ConfigStateMachine stateMachine;

    {
        std::ifstream input_file(specPath);
        cereal::JSONInputArchive iArchive(input_file);
        iArchive(stateMachine);
    }

    return stateMachine;
}
} // namespace fsm
} // namespace kpsr
