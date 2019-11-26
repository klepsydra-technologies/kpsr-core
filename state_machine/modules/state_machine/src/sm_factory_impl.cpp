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

#include <memory>
#include <string>

#include <klepsydra/state_machine/config_state_machine.h>
#include <klepsydra/state_machine/sm_factory_impl.h>
#include <klepsydra/state_machine/state_impl.h>
#include <klepsydra/state_machine/state_machine_impl.h>
#include <klepsydra/state_machine/transition_impl.h>

namespace kpsr {
namespace fsm {

std::shared_ptr<StateMachine>
SMFactoryImpl::createStateMachine(const ConfigStateMachine &cnfSm) {

  std::vector<std::shared_ptr<State>> stateList;
  std::string initialStateId = "";
  for (auto state : cnfSm.states) {
    std::vector<std::shared_ptr<Transition>> transitionList;

    if (initialStateId == "") {
      initialStateId = state.id;
    }

    for (auto transition : state.transitions) {
      transitionList.push_back(std::make_shared<TransitionImpl>(
          transition.destinationId, transition.event));
    }
    stateList.push_back(std::make_shared<StateImpl>(state.id, transitionList));
  }
  return std::make_shared<StateMachineImpl>(cnfSm.id, stateList,
                                            initialStateId);
}
} // namespace fsm
} // namespace kpsr
