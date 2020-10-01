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

#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include <string>

namespace kpsr
{

std::string const DEFAULT_ROOT("");


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
    virtual ~Environment() {}
    /**
     * @brief getPropertyString
     * @param key
     * @param value
     */
    virtual void getPropertyString(const std::string & key, std::string & value, std::string const& rootNode = DEFAULT_ROOT) = 0;

    /**
     * @brief getPropertyInt
     * @param key
     * @param value
     */
    virtual void getPropertyInt(const std::string & key, int & value, std::string const& rootNode = DEFAULT_ROOT) = 0;

    /**
     * @brief getPropertyFloat
     * @param key
     * @param value
     */
    virtual void getPropertyFloat(const std::string & key, float & value, std::string const& rootNode = DEFAULT_ROOT) = 0;

    /**
     * @brief getPropertyBool
     * @param key
     * @param value
     */
    virtual void getPropertyBool(const std::string & key, bool & value, std::string const& rootNode = DEFAULT_ROOT) = 0;

    /**
     * @brief setPropertyString
     * @param key
     * @param value
     */
    virtual void setPropertyString(const std::string & key, const std::string & value, std::string const& rootNode = DEFAULT_ROOT) = 0;

    /**
     * @brief setPropertyInt
     * @param key
     * @param value
     */
    virtual void setPropertyInt(const std::string & key, const int & value, std::string const& rootNode = DEFAULT_ROOT) = 0;

    /**
     * @brief setPropertyFloat
     * @param key
     * @param value
     */
    virtual void setPropertyFloat(const std::string & key, const float & value, std::string const& rootNode = DEFAULT_ROOT) = 0;

    /**
     * @brief setPropertyBool
     * @param key
     * @param value
     */
    virtual void setPropertyBool(const std::string & key, const bool & value, std::string const& rootNode = DEFAULT_ROOT) = 0;

    /**
     * @brief loadFile
     * @param fileName
     * @param nodeName
     *
     * This method is used to load additional configuration data from another file. It might be used in cases where
     * additional data may be loaded later, like in kpsr::YamlEnvironment
     */
    virtual void loadFile(const std::string& fileName, const std::string& nodeName) = 0;

};
}
#endif
