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

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <queue>

#include <klepsydra/state_machine/state.h>
#include <klepsydra/state_machine/state_machine.h>
#include <klepsydra/state_machine/transition.h>

namespace kpsr {
namespace fsm {
class StateMachineImpl : public StateMachine
{
public:
    StateMachineImpl(const std::string &id,
                     const std::vector<std::shared_ptr<State>> &states,
                     const std::string &initialStateId);
    virtual void start() override;
    virtual void stop() override;
    virtual void reset() override;
    virtual void update() override;
    virtual void enqueueEvent(const std::string &event) override;
    virtual void enqueueAndUpdate(const std::string &event) override;
    virtual void registerObserver(
        std::function<void(const std::string &currentState, bool stateChanged)> smObserver) override;
    virtual void unregisterObservers() override;

private:
    std::string _id;
    bool _started;
    std::string _initialStateId;
    std::string _currentStateId;

    std::unordered_map<std::string, std::shared_ptr<State>> _stateTable;
    std::unordered_map<std::string, std::vector<std::shared_ptr<Transition>>> _stateTransitionTable;
    std::vector<std::function<void(const std::string &currentState, bool stateChanged)>> _observers;
    std::queue<std::string> _eventQueue;

    void updateObservers(const std::string &stateId, bool stateChanged);
};
} // namespace fsm
} // namespace kpsr
