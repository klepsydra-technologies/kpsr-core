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

#ifndef ROS_ENV_H
#define ROS_ENV_H

#include "ros/ros.h"

#include <klepsydra/core/environment.h>

namespace kpsr {
namespace ros_mdlw {
/**
 * @brief The RosEnv class
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version 2.0.1
 *
 * @ingroup kpsr-rosstg-composition
 *
 * @details An adaptor from ROS Node to Klepsydra Environment.
 *
 */
class RosEnv : public Environment
{
public:
    /**
     * @brief RosEnv
     * @param nodeHandle
     */
    RosEnv(ros::NodeHandle *nodeHandle);

    /**
     * @brief getPropertyString
     * @param key
     * @param value
     */
    void getPropertyString(const std::string &key,
                           std::string &value,
                           const std::string &rootNode = "");

    /**
     * @brief getPropertyInt
     * @param key
     * @param value
     */
    void getPropertyInt(const std::string &key, int &value, const std::string &rootNode = "");

    /**
     * @brief getPropertyFloat
     * @param key
     * @param value
     */
    void getPropertyFloat(const std::string &key, float &value, const std::string &rootNode = "");

    /**
     * @brief getPropertyBool
     * @param key
     * @param value
     */
    void getPropertyBool(const std::string &key, bool &value, const std::string &rootNode = "");

    /**
     * @brief setPropertyString
     * @param key
     * @param value
     */
    void setPropertyString(const std::string &key,
                           const std::string &value,
                           const std::string &rootNode = "");

    /**
     * @brief setPropertyInt
     * @param key
     * @param value
     */
    void setPropertyInt(const std::string &key, const int &value, const std::string &rootNode = "");

    /**
     * @brief setPropertyFloat
     * @param key
     * @param value
     */
    void setPropertyFloat(const std::string &key,
                          const float &value,
                          const std::string &rootNode = "");

    /**
     * @brief setPropertyBool
     * @param key
     * @param value
     */
    void setPropertyBool(const std::string &key,
                         const bool &value,
                         const std::string &rootNode = "");

    /*
     * @brief loadFile
     * @param fileName
     * @param nodeName
     *
     * This method is used to load additional configuration data from another file. It might be used in cases where
     * additional data may be loaded later, like in kpsr::YamlEnvironment
     *
     * Currently unsupported for ROS
     */

    void loadFile(const std::string &fileName, const std::string &nodeName);

private:
    ros::NodeHandle *nodeHandle;
    std::string getKey(const std::string &key, const std::string &rootNode);
};
} // namespace ros_mdlw
} // namespace kpsr
#endif
