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

#ifndef ENUM_ROS_MAPPER_H
#define ENUM_ROS_MAPPER_H

#include "ros/ros.h"
#include "std_msgs/Int32.h"

#include <klepsydra/serialization/mapper.h>

namespace kpsr
{
template<class E>
/**
 * @brief The Mapper<E, std_msgs::Int32> class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version 2.0.1
 *
 * @ingroup kpsr-rosstg-serialization
 *
 */
class Mapper<E, std_msgs::Int32>
{
public:
    /**
     * @brief fromMiddleware
     * @param environment
     * @param message
     * @param event
     */
    virtual void fromMiddleware(const std_msgs::Int32& message, E& event) {
        event  = (E) message.data;
    }

    /**
     * @brief toMiddleware
     * @param environment
     * @param event
     * @param message
     */
    void toMiddleware(const E& event, std_msgs::Int32& message) {
        message.data = event;
    }
};
}
#endif//ENUM_ROS_MAPPER_H
