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
