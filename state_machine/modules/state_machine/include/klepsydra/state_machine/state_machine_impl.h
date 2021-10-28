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
  virtual void cleanQueue() override;

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
