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

#ifndef IDENTITY_MAPPER_H
#define IDENTITY_MAPPER_H

#include <klepsydra/serialization/mapper.h>

namespace kpsr
{
template <class KpsrClass>
/*!
 * @brief The Mapper class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-serialization
 *
 * @details Main interface to transform from and to a middleware into the Klepsydra realm. Implementations of this class are only needed for non-memory middlewares like ROS and DDS.
*/
class Mapper<KpsrClass, KpsrClass>
{
public:
    /*!
     * @brief fromMiddleware
     * This method converts a middleware message (e.g., ROS message <a href="http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html">geometry_msgs::PoseStamped</a>)
     * into a Klepsydra realm <a href="https://en.wikipedia.org/wiki/Plain_Old_C%2B%2B_Object">POCO</a> (e.g., kpsr::geometry::PoseEventData)
     * @param message
     * @param event
     */
    virtual void fromMiddleware(const KpsrClass& message, KpsrClass& event) {
        event = message;
    }
    /*!
     * @brief toMiddleware
     * This method converts a Klepsydra realm <a href="https://en.wikipedia.org/wiki/Plain_Old_C%2B%2B_Object">POCO</a> (e.g., kpsr::geometry::PoseEventData)
     * into a middleware message (e.g., ROS message <a href="http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html">geometry_msgs::PoseStamped</a>)
     * @param event
     * @param message
     */
    virtual void toMiddleware(const KpsrClass& event, KpsrClass& message) {
        message = event;
    }
};
}
#endif
