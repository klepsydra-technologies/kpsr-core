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

#include <yaml-cpp/yaml.h>

#include <klepsydra/state_machine/yaml_config_loader.h>
#include <klepsydra/state_machine/yaml_to_state_machine.h>

namespace kpsr {
namespace fsm {

ConfigStateMachine YamlConfigLoader::loadConfig(const std::string &specPath) {
  YAML::Node stateMachine = YAML::LoadFile(specPath);
  return stateMachine.as<ConfigStateMachine>();
}
} // namespace fsm
} // namespace kpsr
