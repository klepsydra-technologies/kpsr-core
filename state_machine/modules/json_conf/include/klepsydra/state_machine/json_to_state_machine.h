/*
 * Copyright 2023 Klepsydra Technologies AG
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <cereal/types/string.hpp>
#include <cereal/types/vector.hpp>

#include <klepsydra/state_machine/config_state_machine.h>

namespace cereal {

template<class Archive>
void serialize(Archive &archive, kpsr::fsm::ConfigTransition &transition)
{
    archive(make_nvp("destination", transition.destinationId), make_nvp("event", transition.event));
}

template<class Archive>
void serialize(Archive &archive, kpsr::fsm::ConfigState &state)
{
    archive(make_nvp("id", state.id), make_nvp("transitions", state.transitions));
}

template<class Archive>
void serialize(Archive &archive, kpsr::fsm::ConfigStateMachine &stateMachine)
{
    archive(make_nvp("id", stateMachine.id), make_nvp("states", stateMachine.states));
}

template<>
inline void prologue(JSONInputArchive &, const kpsr::fsm::ConfigStateMachine &)
{}

template<>
inline void epilogue(JSONInputArchive &, const kpsr::fsm::ConfigStateMachine &)
{}

template<>
inline void prologue(JSONOutputArchive &, const kpsr::fsm::ConfigStateMachine &)
{}

template<>
inline void epilogue(JSONOutputArchive &, const kpsr::fsm::ConfigStateMachine &)
{}

} // namespace cereal
