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

#include "ros_env.h"
#include <stdexcept>
#include <string.h>

kpsr::ros_mdlw::RosEnv::RosEnv(ros::NodeHandle *nodeHandle)
{
    this->nodeHandle = nodeHandle;
}

void kpsr::ros_mdlw::RosEnv::getPropertyString(const std::string &key,
                                               std::string &value,
                                               const std::string &rootNode)
{
    auto newKey = getKey(key, rootNode);
    nodeHandle->getParam(newKey, value);
}

void kpsr::ros_mdlw::RosEnv::getPropertyInt(const std::string &key,
                                            int &value,
                                            const std::string &rootNode)
{
    auto newKey = getKey(key, rootNode);
    nodeHandle->getParam(newKey, value);
}

void kpsr::ros_mdlw::RosEnv::getPropertyFloat(const std::string &key,
                                              float &value,
                                              const std::string &rootNode)
{
    auto newKey = getKey(key, rootNode);
    nodeHandle->getParam(newKey, value);
}

void kpsr::ros_mdlw::RosEnv::getPropertyBool(const std::string &key,
                                             bool &value,
                                             const std::string &rootNode)
{
    auto newKey = getKey(key, rootNode);
    nodeHandle->getParam(newKey, value);
}

void kpsr::ros_mdlw::RosEnv::setPropertyString(const std::string &key,
                                               const std::string &value,
                                               const std::string &rootNode)
{
    auto newKey = getKey(key, rootNode);
    nodeHandle->setParam(newKey, value);
}

void kpsr::ros_mdlw::RosEnv::setPropertyInt(const std::string &key,
                                            const int &value,
                                            const std::string &rootNode)
{
    auto newKey = getKey(key, rootNode);
    nodeHandle->setParam(newKey, value);
}

void kpsr::ros_mdlw::RosEnv::setPropertyFloat(const std::string &key,
                                              const float &value,
                                              const std::string &rootNode)
{
    auto newKey = getKey(key, rootNode);
    nodeHandle->setParam(newKey, value);
}

void kpsr::ros_mdlw::RosEnv::setPropertyBool(const std::string &key,
                                             const bool &value,
                                             const std::string &rootNode)
{
    auto newKey = getKey(key, rootNode);
    nodeHandle->setParam(newKey, value);
}

void kpsr::ros_mdlw::RosEnv::loadFile(const std::string &fileName, const std::string &nodeName)
{
    throw "Unsupported operation.";
}

std::string kpsr::ros_mdlw::RosEnv::getKey(const std::string &key, const std::string &rootNode)
{
    std::string newKey(key);
    if (0 != rootNode.size()) {
        newKey = "/" + rootNode + "/" + key;
    }
    return newKey;
}
