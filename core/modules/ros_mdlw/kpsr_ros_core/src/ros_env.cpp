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

#include "ros_env.h"
#include <string.h>
#include <stdexcept>

kpsr::ros_mdlw::RosEnv::RosEnv(ros::NodeHandle * nodeHandle) {
	this->nodeHandle = nodeHandle;
}

void kpsr::ros_mdlw::RosEnv::getPropertyString(const std::string & key, std::string & value, const std::string & rootNode) {
    auto newKey = getKey(key, rootNode);
    nodeHandle->getParam(newKey, value);
}

void kpsr::ros_mdlw::RosEnv::getPropertyInt(const std::string & key, int & value, const std::string & rootNode) {
    auto newKey = getKey(key, rootNode);
    nodeHandle->getParam(newKey, value);
}

void kpsr::ros_mdlw::RosEnv::getPropertyFloat(const std::string & key, float & value, const std::string & rootNode) {
    auto newKey = getKey(key, rootNode);
    nodeHandle->getParam(newKey, value);
}

void kpsr::ros_mdlw::RosEnv::getPropertyBool(const std::string & key, bool & value, const std::string & rootNode) {
    auto newKey = getKey(key, rootNode);
    nodeHandle->getParam(newKey, value);
}

void kpsr::ros_mdlw::RosEnv::setPropertyString(const std::string & key, const std::string & value, const std::string & rootNode) {
    auto newKey = getKey(key, rootNode);
    nodeHandle->setParam(newKey, value);
}

void kpsr::ros_mdlw::RosEnv::setPropertyInt(const std::string & key, const int & value, const std::string & rootNode) {
    auto newKey = getKey(key, rootNode);
    nodeHandle->setParam(newKey, value);
}

void kpsr::ros_mdlw::RosEnv::setPropertyFloat(const std::string & key, const float & value, const std::string & rootNode) {
    auto newKey = getKey(key, rootNode);
    nodeHandle->setParam(newKey, value);
}

void kpsr::ros_mdlw::RosEnv::setPropertyBool(const std::string & key, const bool & value, const std::string & rootNode) {
    auto newKey = getKey(key, rootNode);
    nodeHandle->setParam(newKey, value);
}

void kpsr::ros_mdlw::RosEnv::loadFile(const std::string & fileName, const std::string & nodeName) {
    throw "Unsupported operation.";
}

std::string kpsr::ros_mdlw::RosEnv::getKey(const std::string & key, const std::string & rootNode) {
    std::string newKey(key);
    if (0 != rootNode.size()) {
        newKey = "/" + rootNode + "/" + key;
    }
    return newKey;
}
