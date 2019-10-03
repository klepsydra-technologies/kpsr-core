/****************************************************************************
*
*                           Klepsydra Core Modules
*              Copyright (C) 2019-2020  Klepsydra Technologies GmbH
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*****************************************************************************/

#include <klepsydra/state_machine/state_machine_impl.h>

namespace kpsr {
namespace fsm {

StateMachineImpl::StateMachineImpl(
    const std::string &id, const std::vector<std::shared_ptr<State>> &states,
    const std::string &initialStateId)
    : _id(id), _started(false), _initialStateId(initialStateId),
      _currentStateId(initialStateId) {
  for (auto state : states) {
    _stateTable[state->getId()] = state;
  }
}
void StateMachineImpl::start() {
  _started = true;
  updateObservers(_currentStateId, true);
}
void StateMachineImpl::stop() { _started = false; }
void StateMachineImpl::reset() {
  _started = false;
  _currentStateId = _initialStateId;
}
void StateMachineImpl::update() {
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

void StateMachineImpl::enqueueAndUpdate(const std::string &event) {
  enqueueEvent(event);
  update();
}

void StateMachineImpl::updateObservers(const std::string &stateId,
                                       bool stateChanged) {
  for (auto observer : _observers) {
    observer(_id + ":" + stateId, stateChanged);
  }
}

void StateMachineImpl::enqueueEvent(const std::string &event) {
  if (_started) {
    _eventQueue.push(event);
  }
}

void StateMachineImpl::registerObserver(
    std::function<void(const std::string &currentState, bool stateChanged)>
        smObserver) {
  _observers.push_back(smObserver);
}

void StateMachineImpl::unregisterObservers() { _observers.clear(); }
} // namespace fsm
} // namespace kpsr
