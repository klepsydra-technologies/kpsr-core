/****************************************************************************
*
*                           Klepsydra Core Modules
*              Copyright (C) 2019-2020  Klepsydra Technologies GmbH
*                            All Rights Reserved.
*
*  This file is subject to the terms and conditions defined in
*  file 'LICENSE.md', which is part of this source code package.
*
*  NOTICE:  All information contained herein is, and remains the property of Klepsydra
*  Technologies GmbH and its suppliers, if any. The intellectual and technical concepts
*  contained herein are proprietary to Klepsydra Technologies GmbH and its suppliers and
*  may be covered by Swiss and Foreign Patents, patents in process, and are protected by
*  trade secret or copyright law. Dissemination of this information or reproduction of
*  this material is strictly forbidden unless prior written permission is obtained from
*  Klepsydra Technologies GmbH.
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
