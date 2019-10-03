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

#pragma once

#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>

#include <klepsydra/state_machine/state.h>
#include <klepsydra/state_machine/state_machine.h>
#include <klepsydra/state_machine/transition.h>

namespace kpsr {
namespace fsm {
class StateMachineImpl : public StateMachine {
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
      std::function<void(const std::string &currentState, bool stateChanged)>
          smObserver) override;
  virtual void unregisterObservers() override;

private:
  std::string _id;
  bool _started;
  std::string _initialStateId;
  std::string _currentStateId;

  std::unordered_map<std::string, std::shared_ptr<State>> _stateTable;
  std::unordered_map<std::string, std::vector<std::shared_ptr<Transition>>>
      _stateTransitionTable;
  std::vector<
      std::function<void(const std::string &currentState, bool stateChanged)>>
      _observers;
  std::queue<std::string> _eventQueue;

  void updateObservers(const std::string &stateId, bool stateChanged);
};
} // namespace fsm
} // namespace kpsr
