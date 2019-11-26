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

#include <functional>
#include <unordered_map>

#include <klepsydra/state_machine/state_machine_listener.h>

namespace kpsr {
namespace fsm {
StateMachineListener::StateMachineListener(const std::string &stateMachineId)
    : _stateMachineId(stateMachineId) {}
void StateMachineListener::addAction(
    const std::string &stateId,
    std::function<void(const std::string &)> action) {
  std::string id = getCompleteName(stateId);
  _actions[id].push_back(action);
}

void StateMachineListener::addOneOffAction(
    const std::string &stateId,
    std::function<void(const std::string &)> action) {
  std::string id = getCompleteName(stateId);
  _oneOffActions[id].push_back(action);
}

void StateMachineListener::addPeriodicAction(
    const std::string &stateId,
    std::function<void(const std::string &)> action) {
  std::string id = getCompleteName(stateId);
  _periodicActions[id].push_back(action);
}
void StateMachineListener::removeActions(const std::string &stateId) {
  std::string id = getCompleteName(stateId);
  _periodicActions[id].clear();
}
void StateMachineListener::removeActions() { _periodicActions.clear(); }

void StateMachineListener::updateCurrentState(const std::string &currentStateId,
                                              bool stateChanged) {
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

std::string StateMachineListener::getCompleteName(const std::string &stateId) {
  std::string id = _stateMachineId + ":" + stateId;
  if (_stateMachineId == "") {
    id = stateId;
  }
  return id;
}

std::function<void(const std::string &currentState, bool stateChanged)>
StateMachineListener::getObserverFunc() {
  return std::bind(&StateMachineListener::updateCurrentState, this,
                   std::placeholders::_1, std::placeholders::_2);
}
} // namespace fsm
} // namespace kpsr
