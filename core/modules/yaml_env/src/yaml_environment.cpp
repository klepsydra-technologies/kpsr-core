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
****************************************************************************/

#include <klepsydra/core/yaml_environment.h>

#include <cstring>
#include <fstream>

kpsr::YamlEnvironment::YamlEnvironment(const std::string yamlFileName)
    : _yamlFileName(yamlFileName)
    , _node(YAML::LoadFile(yamlFileName)) {
}

kpsr::YamlEnvironment::YamlEnvironment()
    : _yamlFileName("")
    , _node() {
}

void kpsr::YamlEnvironment::getPropertyString(const std::string key, std::string & value) {
    value = _node[key].as<std::string>();
}

void kpsr::YamlEnvironment::getPropertyInt(const std::string key, int & value) {
    value = _node[key].as<int>();
}

void kpsr::YamlEnvironment::getPropertyFloat(const std::string key, float & value) {
    value = _node[key].as<float>();
}

void kpsr::YamlEnvironment::getPropertyBool(const std::string key, bool & value){
    value = _node[key].as<bool>();
}

void kpsr::YamlEnvironment::setPropertyString(const std::string key, const std::string value){
    _node[key] = value;
}

void kpsr::YamlEnvironment::setPropertyInt(const std::string key, const int & value) {
    _node[key] = value;
}

void kpsr::YamlEnvironment::setPropertyFloat(const std::string key, const float & value) {
    _node[key] = value;
}

void kpsr::YamlEnvironment::setPropertyBool(const std::string key, const bool & value) {
    _node[key] = value;
}

void kpsr::YamlEnvironment::persist() {
    std::ofstream outputFile(_yamlFileName);
    outputFile << _node;;
    outputFile.flush();
    outputFile.close();
}

void kpsr::YamlEnvironment::reload(std::string yamlContent) {
    _node = YAML::Load(yamlContent);
}

std::string kpsr::YamlEnvironment::exportEnvironment() {
    std::stringstream confDataStream;
    confDataStream << _node;
    return confDataStream.str();
}
