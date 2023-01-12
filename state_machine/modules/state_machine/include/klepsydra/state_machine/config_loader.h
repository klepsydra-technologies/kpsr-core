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

#include <klepsydra/state_machine/config_state_machine.h>
#include <string>

namespace kpsr {
namespace fsm {
class ConfigLoader
{
public:
    virtual ConfigStateMachine loadConfig(const std::string &specPath) = 0;
};
} // namespace fsm
} // namespace kpsr
