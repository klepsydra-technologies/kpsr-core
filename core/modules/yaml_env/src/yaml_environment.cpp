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

#include <klepsydra/core/yaml_environment.h>
#include <spdlog/spdlog.h>

#include <cstring>
#include <fstream>

kpsr::YamlEnvironment::YamlEnvironment(const std::string &yamlFileName, const std::string &rootNode)
    : _yamlFileName(yamlFileName)
    , _node()
{
    loadFile(yamlFileName, rootNode);
}

kpsr::YamlEnvironment::YamlEnvironment()
    : _yamlFileName("")
    , _node()
{}

void kpsr::YamlEnvironment::getPropertyString(const std::string &key,
                                              std::string &value,
                                              const std::string &rootNode)
{
    try {
        value = getNode(rootNode)[key].as<std::string>();
    } catch (const YAML::Exception &) {
        spdlog::error("Error getting YAML node for key: {} + {} ", rootNode, key);
        throw;
    }
}

void kpsr::YamlEnvironment::getPropertyInt(const std::string &key,
                                           int &value,
                                           const std::string &rootNode)
{
    try {
        value = getNode(rootNode)[key].as<int>();
    } catch (const YAML::Exception &) {
        spdlog::error("Error getting YAML node for key: {} + {} ", rootNode, key);
        throw;
    }
}

void kpsr::YamlEnvironment::getPropertyFloat(const std::string &key,
                                             float &value,
                                             const std::string &rootNode)
{
    try {
        value = getNode(rootNode)[key].as<float>();
    } catch (const YAML::Exception &) {
        spdlog::error("Error getting YAML node for key: {} + {} ", rootNode, key);
        throw;
    }
}

void kpsr::YamlEnvironment::getPropertyBool(const std::string &key,
                                            bool &value,
                                            const std::string &rootNode)
{
    try {
        value = getNode(rootNode)[key].as<bool>();
    } catch (const YAML::Exception &) {
        spdlog::error("Error getting YAML node for key: {} + {} ", rootNode, key);
        throw;
    }
}

void kpsr::YamlEnvironment::setPropertyString(const std::string &key,
                                              const std::string &value,
                                              const std::string &rootNode)
{
    // Cannot use getNode function here because yaml-cpp does not return correct reference if value is std::string.
    if (rootNode.empty()) {
        _node[key] = value;
    } else {
        _node[rootNode][key] = value;
    }
}

void kpsr::YamlEnvironment::setPropertyInt(const std::string &key,
                                           const int &value,
                                           const std::string &rootNode)
{
    getNode(rootNode)[key] = value;
}

void kpsr::YamlEnvironment::setPropertyFloat(const std::string &key,
                                             const float &value,
                                             const std::string &rootNode)
{
    getNode(rootNode)[key] = value;
}

void kpsr::YamlEnvironment::setPropertyBool(const std::string &key,
                                            const bool &value,
                                            const std::string &rootNode)
{
    getNode(rootNode)[key] = value;
}

void kpsr::YamlEnvironment::updateConfiguration(const std::string &yamlContent,
                                                const std::string &rootNode)
{
    if (rootNode.empty()) {
        updateConfiguration(yamlContent);
    } else {
        _node[rootNode] = YAML::Load(yamlContent);
    }
}

void kpsr::YamlEnvironment::updateConfiguration(const std::string &yamlContent)
{
    _node = YAML::Load(yamlContent);
}

std::string kpsr::YamlEnvironment::exportEnvironment()
{
    std::stringstream confDataStream;
    confDataStream << _node;
    return confDataStream.str();
}

void kpsr::YamlEnvironment::loadFile(const std::string &fileName, const std::string &nodeName)
{
    if (nodeName.empty()) {
        _node = YAML::LoadFile(fileName);
    } else {
        _node[nodeName] = YAML::LoadFile(fileName);
    }
}

YAML::Node kpsr::YamlEnvironment::getNode(const std::string &rootNode)
{
    if (rootNode.empty()) {
        return _node;
    } else {
        return _node[rootNode];
    }
}
