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

#include <functional>
#include <unordered_map>

#include <klepsydra/state_machine/state_machine_listener.h>

namespace kpsr {
namespace fsm {
StateMachineListener::StateMachineListener(const std::string &stateMachineId)
    : _stateMachineId(stateMachineId)
{}
void StateMachineListener::addAction(const std::string &stateId,
                                     std::function<void(const std::string &)> action)
{
    std::string id = getCompleteName(stateId);
    _actions[id].push_back(action);
}

void StateMachineListener::addOneOffAction(const std::string &stateId,
                                           std::function<void(const std::string &)> action)
{
    std::string id = getCompleteName(stateId);
    _oneOffActions[id].push_back(action);
}

void StateMachineListener::addPeriodicAction(const std::string &stateId,
                                             std::function<void(const std::string &)> action)
{
    std::string id = getCompleteName(stateId);
    _periodicActions[id].push_back(action);
}
void StateMachineListener::removeActions(const std::string &stateId)
{
    std::string id = getCompleteName(stateId);
    _periodicActions[id].clear();
}
void StateMachineListener::removeActions()
{
    _periodicActions.clear();
}

void StateMachineListener::updateCurrentState(const std::string &currentStateId, bool stateChanged)
{
    if (stateChanged) {
        if (_oneOffActions.find(currentStateId) != _oneOffActions.end()) {
            for (auto action : _oneOffActions[currentStateId]) {
                action(currentStateId);
            }
            _oneOffActions.erase(currentStateId);
        }
        if (_actions.find(currentStateId) != _actions.end()) {
            for (auto action : _actions[currentStateId]) {
                action(currentStateId);
            }
        }
    }
    if (_periodicActions.find(currentStateId) != _periodicActions.end()) {
        for (auto action : _periodicActions[currentStateId]) {
            action(currentStateId);
        }
    }
}

std::string StateMachineListener::getCompleteName(const std::string &stateId)
{
    std::string id = _stateMachineId + ":" + stateId;
    if (_stateMachineId == "") {
        id = stateId;
    }
    return id;
}

std::function<void(const std::string &currentState, bool stateChanged)>
StateMachineListener::getObserverFunc()
{
    return std::bind(&StateMachineListener::updateCurrentState,
                     this,
                     std::placeholders::_1,
                     std::placeholders::_2);
}
} // namespace fsm
} // namespace kpsr
