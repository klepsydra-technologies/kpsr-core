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

#include <string>

#include <klepsydra/state_machine/transition.h>

namespace kpsr {
namespace fsm {
class TransitionImpl : public Transition
{
public:
    TransitionImpl(const std::string &destinationStateId, const std::string &event);
    virtual bool canTransition(const std::string &event) override;
    virtual std::string getDestinationStateId() override;

private:
    std::string _destStateId;
    std::string _eventId;
};
} // namespace fsm
} // namespace kpsr
