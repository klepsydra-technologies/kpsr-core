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

#include <string>

#include <klepsydra/state_machine/transition.h>

namespace kpsr {
namespace fsm {
class TransitionImpl : public Transition {
public:
  TransitionImpl(const std::string &destinationStateId,
                 const std::string &event);
  virtual bool canTransition(const std::string &event) override;
  virtual std::string getDestinationStateId() override;

private:
  std::string _destStateId;
  std::string _eventId;
};
} // namespace fsm
} // namespace kpsr
