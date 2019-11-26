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

#include <functional>
#include <unordered_map>
#include <vector>

#include <klepsydra/state_machine/state_machine.h>

namespace kpsr {
namespace fsm {
class StateMachineListener {
public:
  StateMachineListener(const std::string &stateMachineId = "");

  std::function<void(const std::string &currentState, bool stateChanged)>
  getObserverFunc();

  void addAction(const std::string &stateId,
                 std::function<void(const std::string &)> action);
  void addOneOffAction(const std::string &stateId,
                       std::function<void(const std::string &)> action);
  void addPeriodicAction(const std::string &stateId,
                         std::function<void(const std::string &)> action);
  void removeActions(const std::string &stateId);
  void removeActions();

private:
  void updateCurrentState(const std::string &currentStateId, bool stateChanged);
  std::unordered_map<std::string,
                     std::vector<std::function<void(const std::string &)>>>
      _actions;
  std::unordered_map<std::string,
                     std::vector<std::function<void(const std::string &)>>>
      _oneOffActions;
  std::unordered_map<std::string,
                     std::vector<std::function<void(const std::string &)>>>
      _periodicActions;
  std::string _stateMachineId;

  std::string getCompleteName(const std::string &stateId);
};
} // namespace fsm
} // namespace kpsr
