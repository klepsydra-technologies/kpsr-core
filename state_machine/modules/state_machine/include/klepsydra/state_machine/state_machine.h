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

#include <functional>
#include <memory>
#include <string>

namespace kpsr {
namespace fsm {
class StateMachine {
public:
  virtual void start() = 0;
  virtual void stop() = 0;
  virtual void update() = 0;
  virtual void enqueueAndUpdate(const std::string &event) = 0;
  virtual void reset() = 0;
  virtual void enqueueEvent(const std::string &event) = 0;
  virtual void registerObserver(
      std::function<void(const std::string &currentState, bool stateChanged)>
          smObserver) = 0;
  virtual void unregisterObservers() = 0;
};
} // namespace fsm
} // namespace kpsr
