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

#include "persistent_ros_env.h"
#include <string.h>

#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h"

kpsr::ros_mdlw::PersistentRosEnv::PersistentRosEnv(ros::NodeHandle * nodeHandle, const std::string yamlFileName, const PersitancePolicy persitancePolicy) {
    _nodeHandle = nodeHandle;
    _persistentEnv = new YamlEnvironment(yamlFileName);
    _persitancePolicy = persitancePolicy;
}

void kpsr::ros_mdlw::PersistentRosEnv::getPropertyString(const std::string key, std::string & value) {
    _nodeHandle->getParam(key, value);

    if (_persitancePolicy == ON_GET) {
        try {
            std::string persistedValue;
            _persistentEnv->getPropertyString(key, persistedValue);
            if (value.compare(persistedValue) != 0) {
                _persistentEnv->setPropertyString(key, value);
                _persistentEnv->persist();
            }
        } catch (const YAML::TypedBadConversion<std::string>&) {
            spdlog::info("kpsr::ros_mdlw::PersistentRosEnv::getPropertyString. Property {} does not exist.", key);
        }
    }
}

void kpsr::ros_mdlw::PersistentRosEnv::getPropertyInt(const std::string key, int & value) {
    _nodeHandle->getParam(key, value);

    if (_persitancePolicy == ON_GET) {
        try {
            int persistedValue;
            _persistentEnv->getPropertyInt(key, persistedValue);
            if (value != persistedValue) {
                _persistentEnv->setPropertyInt(key, value);
                _persistentEnv->persist();
            }
        } catch (const YAML::TypedBadConversion<int>&) {
            spdlog::info("kpsr::ros_mdlw::PersistentRosEnv::getPropertyInt. Property {} does not exist.", key);
        }
    }
}

void kpsr::ros_mdlw::PersistentRosEnv::getPropertyFloat(const std::string key, float & value) {
    _nodeHandle->getParam(key, value);

    if (_persitancePolicy == ON_GET) {
        try {
            float persistedValue;
            _persistentEnv->getPropertyFloat(key, persistedValue);
            if (value != persistedValue) {
                _persistentEnv->setPropertyFloat(key, value);
                _persistentEnv->persist();
            }
        } catch (const YAML::TypedBadConversion<float>&) {
            spdlog::info("kpsr::ros_mdlw::PersistentRosEnv::getPropertyFloat. Property {} does not exist.", key);
        }
    }
}

void kpsr::ros_mdlw::PersistentRosEnv::getPropertyBool(const std::string key, bool & value) {
    _nodeHandle->getParam(key, value);

    if (_persitancePolicy == ON_GET) {
        try {
            bool persistedValue;
            _persistentEnv->getPropertyBool(key, persistedValue);
            if (value != persistedValue) {
                _persistentEnv->setPropertyBool(key, value);
                _persistentEnv->persist();
            }
        } catch (const YAML::TypedBadConversion<bool>&) {
            spdlog::info("kpsr::ros_mdlw::PersistentRosEnv::getPropertyBool. Property {} does not exist.", key);
        }
    }
}

void kpsr::ros_mdlw::PersistentRosEnv::setPropertyString(const std::string key, const std::string value) {
    _nodeHandle->setParam(key, value);

    if (_persitancePolicy == ON_SET) {
        _persistentEnv->setPropertyString(key, value);
        _persistentEnv->persist();
    }
}

void kpsr::ros_mdlw::PersistentRosEnv::setPropertyInt(const std::string key, const int & value) {
    _nodeHandle->setParam(key, value);

    if (_persitancePolicy == ON_SET) {
        _persistentEnv->setPropertyInt(key, value);
        _persistentEnv->persist();
    }
}

void kpsr::ros_mdlw::PersistentRosEnv::setPropertyFloat(const std::string key, const float & value) {
    _nodeHandle->setParam(key, value);

    if (_persitancePolicy == ON_SET) {
        _persistentEnv->setPropertyFloat(key, value);
        _persistentEnv->persist();
    }
}

void kpsr::ros_mdlw::PersistentRosEnv::setPropertyBool(const std::string key, const bool & value) {
    _nodeHandle->setParam(key, value);

    if (_persitancePolicy == ON_SET) {
        _persistentEnv->setPropertyBool(key, value);
        _persistentEnv->persist();
    }
}

void kpsr::ros_mdlw::PersistentRosEnv::persist() {
    if (_persitancePolicy != NONE) {
        _persistentEnv->persist();
    }
}
