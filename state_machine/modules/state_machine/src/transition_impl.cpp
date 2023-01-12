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

#include <klepsydra/state_machine/transition_impl.h>

namespace kpsr {
namespace fsm {

TransitionImpl::TransitionImpl(const std::string &destinationStateId, const std::string &event)
    : _destStateId(destinationStateId)
    , _eventId(event)
{}
bool TransitionImpl::canTransition(const std::string &event)
{
    return event == _eventId;
}
std::string TransitionImpl::getDestinationStateId()
{
    return _destStateId;
}

} // namespace fsm
} // namespace kpsr
