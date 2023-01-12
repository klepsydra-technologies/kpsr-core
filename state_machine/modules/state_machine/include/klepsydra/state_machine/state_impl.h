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

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <klepsydra/state_machine/state.h>
#include <klepsydra/state_machine/transition.h>

namespace kpsr {
namespace fsm {
class StateImpl : public State
{
public:
    StateImpl(const std::string &id, std::vector<std::shared_ptr<Transition>> transitions);
    virtual std::string getId() override;
    virtual std::vector<std::shared_ptr<Transition>> getTransitions() override;

private:
    std::string _id;
    std::vector<std::shared_ptr<Transition>> _transitions;
};
} // namespace fsm
} // namespace kpsr
