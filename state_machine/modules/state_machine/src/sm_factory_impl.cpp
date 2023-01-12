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

#include <memory>
#include <string>

#include <klepsydra/state_machine/config_state_machine.h>
#include <klepsydra/state_machine/sm_factory_impl.h>
#include <klepsydra/state_machine/state_impl.h>
#include <klepsydra/state_machine/state_machine_impl.h>
#include <klepsydra/state_machine/transition_impl.h>

namespace kpsr {
namespace fsm {

std::shared_ptr<StateMachine> SMFactoryImpl::createStateMachine(const ConfigStateMachine &cnfSm)
{
    std::vector<std::shared_ptr<State>> stateList;
    std::string initialStateId = "";
    for (auto state : cnfSm.states) {
        std::vector<std::shared_ptr<Transition>> transitionList;

        if (initialStateId == "") {
            initialStateId = state.id;
        }

        for (auto transition : state.transitions) {
            transitionList.push_back(
                std::make_shared<TransitionImpl>(transition.destinationId, transition.event));
        }
        stateList.push_back(std::make_shared<StateImpl>(state.id, transitionList));
    }
    return std::make_shared<StateMachineImpl>(cnfSm.id, stateList, initialStateId);
}
} // namespace fsm
} // namespace kpsr
