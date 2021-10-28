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
  virtual void cleanQueue() = 0;
};
} // namespace fsm
} // namespace kpsr
