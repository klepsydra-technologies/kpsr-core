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
