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

#ifndef TO_ROS_CHANNEL_H
#define TO_ROS_CHANNEL_H

#include "ros/ros.h"

#include <klepsydra/core/object_pool_publisher.h>

namespace kpsr {
namespace ros_mdlw {
template<class M>
/**
 * @brief The ToRosChannel class
 *
 * @copyright 2023 Klepsydra Technologies AG
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
    ToRosChannel(Container *container,
                 const std::string &name,
                 int poolSize,
                 std::function<void(M &)> initializerFunction,
                 ros::Publisher &rosPublisher)
        : ObjectPoolPublisher<M>(container, name, "ROS", poolSize, initializerFunction, nullptr)
        , _rosPublisher(rosPublisher)
    {}

protected:
    void internalPublish(std::shared_ptr<const M> message) override
    {
        _rosPublisher.publish(*message);
    }

private:
    ros::Publisher &_rosPublisher;
};
} // namespace ros_mdlw
} // namespace kpsr
#endif
