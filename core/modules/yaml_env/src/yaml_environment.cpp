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
****************************************************************************/

#include <klepsydra/core/yaml_environment.h>

#include <cstring>
#include <fstream>

kpsr::YamlEnvironment::YamlEnvironment(const std::string yamlFileName, std::string const& rootNode)
    : _yamlFileName(yamlFileName)
    , _node() {

    _node[rootNode] = YAML::LoadFile(yamlFileName);
}

kpsr::YamlEnvironment::YamlEnvironment()
    : _yamlFileName("")
    , _node() {
}

void kpsr::YamlEnvironment::getPropertyString(const std::string& key, std::string & value, std::string const& rootNode) {
    value = _node[rootNode][key].as<std::string>();
}

void kpsr::YamlEnvironment::getPropertyInt(const std::string& key, int & value, std::string const& rootNode) {
    value = _node[rootNode][key].as<int>();
}

void kpsr::YamlEnvironment::getPropertyFloat(const std::string& key, float & value, std::string const& rootNode) {
    value = _node[rootNode][key].as<float>();
}

void kpsr::YamlEnvironment::getPropertyBool(const std::string& key, bool & value, std::string const& rootNode) {
    value = _node[rootNode][key].as<bool>();
}

void kpsr::YamlEnvironment::setPropertyString(const std::string& key, const std::string value, std::string const& rootNode){
    _node[rootNode][key] = value;
}

void kpsr::YamlEnvironment::setPropertyInt(const std::string& key, const int & value, std::string const& rootNode) {
    _node[rootNode][key] = value;
}

void kpsr::YamlEnvironment::setPropertyFloat(const std::string& key, const float & value, std::string const& rootNode) {
    _node[rootNode][key] = value;
}

void kpsr::YamlEnvironment::setPropertyBool(const std::string& key, const bool & value, std::string const& rootNode) {
    _node[rootNode][key] = value;
}

void kpsr::YamlEnvironment::updateConfiguration(std::string yamlContent, std::string const& rootNode) {
    _node[rootNode] = YAML::Load(yamlContent);
}

void kpsr::YamlEnvironment::updateConfiguration(std::string yamlContent) {
    _node = YAML::Load(yamlContent);
}

std::string kpsr::YamlEnvironment::exportEnvironment() {
    std::stringstream confDataStream;
    confDataStream << _node;
    return confDataStream.str();
}

void kpsr::YamlEnvironment::loadFile(const std::string& fileName, const std::string& nodeName) {
    _node[nodeName] = YAML::LoadFile(fileName);
}
