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

#include <yaml-cpp/yaml.h>

#include <klepsydra/state_machine/config_state_machine.h>

namespace YAML {
template<>
struct convert<kpsr::fsm::ConfigTransition>
{
    static Node encode(const kpsr::fsm::ConfigTransition &transition)
    {
        Node node;
        node["destination"] = transition.destinationId;
        node["event"] = transition.event;
        return node;
    }

    static bool decode(const Node &node, kpsr::fsm::ConfigTransition &transition)
    {
        transition.destinationId = node["destination"].as<std::string>();
        transition.event = node["event"].as<std::string>();
        return true;
    }
};

template<>
struct convert<kpsr::fsm::ConfigState>
{
    static Node encode(const kpsr::fsm::ConfigState &state)
    {
        Node node;
        node["id"] = state.id;
        for (auto transition : state.transitions) {
            Node configTransition;
            configTransition["transition"] = transition;
            node["transitions"].push_back(configTransition);
        }
        return node;
    }

    static bool decode(const Node &node, kpsr::fsm::ConfigState &state)
    {
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

template<>
struct convert<kpsr::fsm::ConfigStateMachine>
{
    static Node encode(const kpsr::fsm::ConfigStateMachine &stateMachine)
    {
        Node node;
        node["id"] = stateMachine.id;
        for (auto state : stateMachine.states) {
            Node configState;
            configState["state"] = state;
            node["states"].push_back(configState);
        }
        return node;
    }

    static bool decode(const Node &node, kpsr::fsm::ConfigStateMachine &stateMachine)
    {
        Node sm = node["state_machine"];
        stateMachine.id = sm["id"].as<std::string>();
        for (auto state : sm["states"]) {
            stateMachine.states.push_back(state["state"].as<kpsr::fsm::ConfigState>());
        }
        return true;
    }
};
} // namespace YAML
