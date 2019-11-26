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
#include <vector>

#include <klepsydra/state_machine/state.h>
#include <klepsydra/state_machine/transition.h>

namespace kpsr {
namespace fsm {
class StateImpl : public State {
public:
  StateImpl(const std::string &id,
            std::vector<std::shared_ptr<Transition>> transitions);
  virtual std::string getId() override;
  virtual std::vector<std::shared_ptr<Transition>> getTransitions() override;

private:
  std::string _id;
  std::vector<std::shared_ptr<Transition>> _transitions;
};
} // namespace fsm
} // namespace kpsr
