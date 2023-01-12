/*
 * Copyright 2023 Klepsydra Technologies AG
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <klepsydra/state_machine/sm_factory_impl.h>
#include <klepsydra/state_machine/state_machine_listener.h>
#include <klepsydra/state_machine/yaml_config_loader.h>

namespace kpsr {
namespace fsm {
class FromYaml
{
public:
    static std::shared_ptr<StateMachine> createStateMachine(const std::string &specPath);
    static std::shared_ptr<StateMachineListener> createStateMachineListener(
        const std::string &specPath);
};
} // namespace fsm
} // namespace kpsr
