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

#include <yaml-cpp/yaml.h>

#include <klepsydra/state_machine/config_state_machine.h>

namespace YAML {
template <> struct convert<kpsr::fsm::ConfigTransition> {
  static Node encode(const kpsr::fsm::ConfigTransition &transition) {
    Node node;
    node["destination"] = transition.destinationId;
    node["event"] = transition.event;
    return node;
  }

  static bool decode(const Node &node,
                     kpsr::fsm::ConfigTransition &transition) {
    transition.destinationId = node["destination"].as<std::string>();
    transition.event = node["event"].as<std::string>();
    return true;
  }
};

template <> struct convert<kpsr::fsm::ConfigState> {
  static Node encode(const kpsr::fsm::ConfigState &state) {
    Node node;
    node["id"] = state.id;
    for (auto transition : state.transitions) {
      Node configTransition;
      configTransition["transition"] = transition;
      node["transitions"].push_back(configTransition);
    }
    return node;
  }

  static bool decode(const Node &node, kpsr::fsm::ConfigState &state) {
    state.id = node["id"].as<std::string>();
    if (node["transitions"]) {
      for (auto transition : node["transitions"]) {
        state.transitions.push_back(
            transition["transition"].as<kpsr::fsm::ConfigTransition>());
      }
    }
    return true;
  }
};

template <> struct convert<kpsr::fsm::ConfigStateMachine> {
  static Node encode(const kpsr::fsm::ConfigStateMachine &stateMachine) {
    Node node;
    node["id"] = stateMachine.id;
    for (auto state : stateMachine.states) {
      Node configState;
      configState["state"] = state;
      node["states"].push_back(configState);
    }
    return node;
  }

  static bool decode(const Node &node,
                     kpsr::fsm::ConfigStateMachine &stateMachine) {
    Node sm = node["state_machine"];
    stateMachine.id = sm["id"].as<std::string>();
    for (auto state : sm["states"]) {
      stateMachine.states.push_back(
          state["state"].as<kpsr::fsm::ConfigState>());
    }
    return true;
  }
};
} // namespace YAML
