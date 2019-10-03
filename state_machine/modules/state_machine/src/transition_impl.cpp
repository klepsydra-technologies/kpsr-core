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

#include <klepsydra/state_machine/transition_impl.h>

namespace kpsr {
namespace fsm {

TransitionImpl::TransitionImpl(const std::string &destinationStateId,
                               const std::string &event)
    : _destStateId(destinationStateId), _eventId(event) {}
bool TransitionImpl::canTransition(const std::string &event) {
  return event == _eventId;
}
std::string TransitionImpl::getDestinationStateId() { return _destStateId; }

} // namespace fsm
} // namespace kpsr
