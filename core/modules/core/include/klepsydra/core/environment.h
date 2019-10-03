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

#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include <string>

namespace kpsr
{

/**
 * @brief The Environment interface.
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-application
 *
 * @details This class is a facility class that was born to isolate the ROS environment from the application classes so that they can stay agnostic of ROS.
 * There are several implementations including a mock one based on memory maps that can be used for unit testing.
 *
*/

class Environment
{
public:
    /**
     * @brief getPropertyString
     * @param key
     * @param value
     */
    virtual void getPropertyString(const std::string key, std::string & value) = 0;

    /**
     * @brief getPropertyInt
     * @param key
     * @param value
     */
    virtual void getPropertyInt(const std::string key, int & value) = 0;

    /**
     * @brief getPropertyFloat
     * @param key
     * @param value
     */
    virtual void getPropertyFloat(const std::string key, float & value) = 0;

    /**
     * @brief getPropertyBool
     * @param key
     * @param value
     */
    virtual void getPropertyBool(const std::string key, bool & value) = 0;

    /**
     * @brief setPropertyString
     * @param key
     * @param value
     */
    virtual void setPropertyString(const std::string key, const std::string value) = 0;

    /**
     * @brief setPropertyInt
     * @param key
     * @param value
     */
    virtual void setPropertyInt(const std::string key, const int & value) = 0;

    /**
     * @brief setPropertyFloat
     * @param key
     * @param value
     */
    virtual void setPropertyFloat(const std::string key, const float & value) = 0;

    /**
     * @brief setPropertyBool
     * @param key
     * @param value
     */
    virtual void setPropertyBool(const std::string key, const bool & value) = 0;

    /**
     * @brief persist
     *
     * This method is used to invoke persistence of the properties. I might be used in cases where
     * the persistance does not happen automatically, for example in the kpsr::YamlEnvironment.
     */
    virtual void persist() = 0;
};
}
#endif
