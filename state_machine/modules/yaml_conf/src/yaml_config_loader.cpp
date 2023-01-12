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

#include <yaml-cpp/yaml.h>

#include <klepsydra/state_machine/yaml_config_loader.h>
#include <klepsydra/state_machine/yaml_to_state_machine.h>

namespace kpsr {
namespace fsm {

ConfigStateMachine YamlConfigLoader::loadConfig(const std::string &specPath)
{
    YAML::Node stateMachine = YAML::LoadFile(specPath);
    return stateMachine.as<ConfigStateMachine>();
}
} // namespace fsm
} // namespace kpsr
