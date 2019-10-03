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
