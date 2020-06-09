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

#ifndef TO_ROS_CHANNEL_H
#define TO_ROS_CHANNEL_H

#include "ros/ros.h"

#include <klepsydra/core/object_pool_publisher.h>

namespace kpsr
{
namespace ros_mdlw
{
template<class M>
/**
 * @brief The ToRosChannel class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version 2.0.1
 *
 * @ingroup kpsr-rosstg-internal
 *
 * Klepsydra to ROS middleware adapter or channel.
 *
 */
class ToRosChannel : public ObjectPoolPublisher<M>
{
public:
    /**
     * @brief ToRosChannel
     * @param environment
     * @param rosPublisher
     */
    ToRosChannel(Container * container, const std::string & name, int poolSize, std::function<void(M &)> initializerFunction, ros::Publisher & rosPublisher)
        : ObjectPoolPublisher<M>(container, name, "ROS", poolSize, initializerFunction, nullptr)
        , _rosPublisher(rosPublisher)
    {}

protected:
    void internalPublish(std::shared_ptr<const M> message) override {
        _rosPublisher.publish(*message);
    }
private:
    ros::Publisher & _rosPublisher;
};
}
}
#endif
