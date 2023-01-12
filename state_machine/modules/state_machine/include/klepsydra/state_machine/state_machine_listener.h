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
#include <unordered_map>
#include <vector>

#include <klepsydra/state_machine/state_machine.h>

namespace kpsr {
namespace fsm {
class StateMachineListener
{
public:
    StateMachineListener(const std::string &stateMachineId = "");

    std::function<void(const std::string &currentState, bool stateChanged)> getObserverFunc();

    void addAction(const std::string &stateId, std::function<void(const std::string &)> action);
    void addOneOffAction(const std::string &stateId,
                         std::function<void(const std::string &)> action);
    void addPeriodicAction(const std::string &stateId,
                           std::function<void(const std::string &)> action);
    void removeActions(const std::string &stateId);
    void removeActions();

private:
    void updateCurrentState(const std::string &currentStateId, bool stateChanged);
    std::unordered_map<std::string, std::vector<std::function<void(const std::string &)>>> _actions;
    std::unordered_map<std::string, std::vector<std::function<void(const std::string &)>>>
        _oneOffActions;
    std::unordered_map<std::string, std::vector<std::function<void(const std::string &)>>>
        _periodicActions;
    std::string _stateMachineId;

    std::string getCompleteName(const std::string &stateId);
};
} // namespace fsm
} // namespace kpsr
