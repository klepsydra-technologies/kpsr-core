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

#include "persistent_ros_env.h"
#include <string.h>

#include <spdlog/spdlog.h>


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
