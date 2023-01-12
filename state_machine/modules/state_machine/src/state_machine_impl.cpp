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

#include <klepsydra/state_machine/state_machine_impl.h>

namespace kpsr {
namespace fsm {

StateMachineImpl::StateMachineImpl(const std::string &id,
                                   const std::vector<std::shared_ptr<State>> &states,
                                   const std::string &initialStateId)
    : _id(id)
    , _started(false)
    , _initialStateId(initialStateId)
    , _currentStateId(initialStateId)
{
    for (auto state : states) {
        _stateTable[state->getId()] = state;
    }
}
void StateMachineImpl::start()
{
    _started = true;
    updateObservers(_currentStateId, true);
}
void StateMachineImpl::stop()
{
    _started = false;
}
void StateMachineImpl::reset()
{
    _started = false;
    _currentStateId = _initialStateId;
}
void StateMachineImpl::update()
{
    bool stateChanged = false;
    if (_started) {
        if (_stateTable.find(_currentStateId) != _stateTable.end()) {
            while (!_eventQueue.empty()) {
                std::string event = _eventQueue.front();
                _eventQueue.pop();
                for (auto t : _stateTable[_currentStateId]->getTransitions()) {
                    if (t->canTransition(event)) {
                        _currentStateId = t->getDestinationStateId();
                        stateChanged = true;
                        break;
                    }
                }
                if (stateChanged)
                    break;
            }
            updateObservers(_currentStateId, stateChanged);
        }
    }
}

void StateMachineImpl::enqueueAndUpdate(const std::string &event)
{
    enqueueEvent(event);
    update();
}

void StateMachineImpl::updateObservers(const std::string &stateId, bool stateChanged)
{
    for (auto observer : _observers) {
        observer(_id + ":" + stateId, stateChanged);
    }
}

void StateMachineImpl::enqueueEvent(const std::string &event)
{
    if (_started) {
        _eventQueue.push(event);
    }
}

void StateMachineImpl::registerObserver(
    std::function<void(const std::string &currentState, bool stateChanged)> smObserver)
{
    _observers.push_back(smObserver);
}

void StateMachineImpl::unregisterObservers()
{
    _observers.clear();
}
} // namespace fsm
} // namespace kpsr
