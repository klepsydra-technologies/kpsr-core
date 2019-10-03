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
    ToRosChannel(Container * container, std::string name, int poolSize, std::function<void(M &)> initializerFunction, ros::Publisher & rosPublisher)
        : ObjectPoolPublisher<M>(container, name, "ROS", poolSize, initializerFunction, nullptr)
        , _rosPublisher(rosPublisher)
    {}

protected:
    void internalPublish(std::shared_ptr<const M> message) override {
        _rosPublisher.publish(* message.get());
    }
private:
    ros::Publisher & _rosPublisher;
};
}
}
#endif
