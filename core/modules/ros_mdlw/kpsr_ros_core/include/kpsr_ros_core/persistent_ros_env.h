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

#ifndef PERSISTENT_ROS_ENV_H
#define PERSISTENT_ROS_ENV_H

#include "ros/ros.h"

#include <klepsydra/core/environment.h>
#include <klepsydra/core/yaml_environment.h>

namespace kpsr
{
namespace ros_mdlw
{
/**
 * @brief The PersistentRosEnv class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version 2.0.1
 *
 * @ingroup kpsr-rosstg-composition
 *
 * @details Ros environment that includes a persistent facility to store the properties in a yaml file.
 * This file can then be read on startup. The way this works is that when a property is set through
 * the Klepsydra Environment class, ROS is updated via setParam and also a YAML file as well.
 *
 */
class PersistentRosEnv : public Environment
{
public:
   /**
    * @brief The PersitancePolicy enum. List of persistance policies to save the YAML file.
    */
    enum PersitancePolicy
    {
        NONE = 0,
        ON_GET,
        ON_SET,
        ON_PERSIST
    };

    /**
     * @brief PersistentRosEnv
     * @param nodeHandle
     * @param yamlFileName
     * @param persitancePolicy
     */
    PersistentRosEnv(ros::NodeHandle * nodeHandle, const std::string yamlFileName, const PersitancePolicy persitancePolicy);

    /**
     * @brief getPropertyString
     * @param key
     * @param value
     */
    void getPropertyString(const std::string key, std::string & value);

    /**
     * @brief getPropertyInt
     * @param key
     * @param value
     */
    void getPropertyInt(const std::string key, int & value);

    /**
     * @brief getPropertyFloat
     * @param key
     * @param value
     */
    void getPropertyFloat(const std::string key, float & value);

    /**
     * @brief getPropertyBool
     * @param key
     * @param value
     */
    void getPropertyBool(const std::string key, bool & value);

    /**
     * @brief setPropertyString
     * @param key
     * @param value
     */
    void setPropertyString(const std::string key, const std::string value);

    /**
     * @brief setPropertyInt
     * @param key
     * @param value
     */
    void setPropertyInt(const std::string key, const int & value);

    /**
     * @brief setPropertyFloat
     * @param key
     * @param value
     */
    void setPropertyFloat(const std::string key, const float & value);

    /**
     * @brief setPropertyBool
     * @param key
     * @param value
     */
    void setPropertyBool(const std::string key, const bool & value);

    /**
     * @brief persist YAML file persist method.
     */
    void persist();

private:
    ros::NodeHandle * _nodeHandle;
    YamlEnvironment * _persistentEnv;
    PersitancePolicy _persitancePolicy;

};
}
}
#endif
