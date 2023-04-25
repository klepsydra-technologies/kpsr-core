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

#include <klepsydra/state_machine/json_config_loader.h>
#include <klepsydra/state_machine/kpsr_state_machine.h>
#include <klepsydra/state_machine/sm_factory_impl.h>

namespace kpsr {
namespace fsm {
std::shared_ptr<StateMachine> FromJson::createStateMachine(const std::string &specPath)
{
    JsonConfigLoader cnfLoader;
    ConfigStateMachine cnfSm = cnfLoader.loadConfig(specPath);
    SMFactoryImpl smFactory;
    return smFactory.createStateMachine(cnfSm);
}

std::shared_ptr<StateMachineListener> FromJson::createStateMachineListener(
    const std::string &specPath)
{
    JsonConfigLoader cnfLoader;
    ConfigStateMachine cnfSm = cnfLoader.loadConfig(specPath);
    return std::make_shared<StateMachineListener>(cnfSm.id);
}
} // namespace fsm
} // namespace kpsr
