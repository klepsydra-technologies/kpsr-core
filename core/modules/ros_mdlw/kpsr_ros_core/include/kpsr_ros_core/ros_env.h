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

#ifndef ROS_ENV_H
#define ROS_ENV_H

#include "ros/ros.h"

#include <klepsydra/core/environment.h>

namespace kpsr
{
namespace ros_mdlw
{
/**
 * @brief The RosEnv class
 *
 * @copyright Klepsydra Technologies 2019-2020.
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
    RosEnv(ros::NodeHandle * nodeHandle);

    /**
     * @brief getPropertyString
     * @param key
     * @param value
     */
    void getPropertyString(const std::string & key, std::string & value, const std::string & rootNode="");

    /**
     * @brief getPropertyInt
     * @param key
     * @param value
     */
    void getPropertyInt(const std::string & key, int & value, const std::string & rootNode="");

    /**
     * @brief getPropertyFloat
     * @param key
     * @param value
     */
    void getPropertyFloat(const std::string & key, float & value, const std::string & rootNode="");

    /**
     * @brief getPropertyBool
     * @param key
     * @param value
     */
    void getPropertyBool(const std::string & key, bool & value, const std::string & rootNode="");

    /**
     * @brief setPropertyString
     * @param key
     * @param value
     */
    void setPropertyString(const std::string & key, const std::string & value, const std::string & rootNode="");

    /**
     * @brief setPropertyInt
     * @param key
     * @param value
     */
    void setPropertyInt(const std::string & key, const int & value, const std::string & rootNode="");

    /**
     * @brief setPropertyFloat
     * @param key
     * @param value
     */
    void setPropertyFloat(const std::string & key, const float & value, const std::string & rootNode="");

    /**
     * @brief setPropertyBool
     * @param key
     * @param value
     */
    void setPropertyBool(const std::string & key, const bool & value, const std::string & rootNode="");

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

    void loadFile(const std::string & fileName, const std::string & nodeName);


private:
    ros::NodeHandle * nodeHandle;
    std::string getKey(const std::string & key, const std::string & rootNode);
};
}
}
#endif
