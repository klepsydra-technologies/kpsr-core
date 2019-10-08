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

#ifndef TO_ROS_MIDDLEWARE_PROVIDER_H
#define TO_ROS_MIDDLEWARE_PROVIDER_H

#include <map>
#include <memory>

#include <ros/ros.h>

#include <klepsydra/core/to_middleware_channel.h>
#include "to_ros_channel.h"

namespace kpsr {
namespace ros_mdlw {
/**
 * @brief The ToRosMiddlewareProvider class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version 2.0.1
 *
 * @ingroup kpsr-rosstg-composition
 *
 * @details This class is a wizard for cretion of Klepsydra to ROS publishers. Its use is very straightforward
 * as it is shown in the following example:
@code
    // Initializing ros
    ros::init(argc, argv, "kpsr_ros_core_test");
    ros::NodeHandle nodeHandle;
    ros::Rate rate(100);

    // Get a ros publisher instance
    ros::Publisher stringPublisher = nodeHandle.advertise<std_msgs::String>("kpsr_ros_core_test_topic", 1);

    // Create the main to ros wizard instance. One for the whole application.
    kpsr::ros_mdlw::ToRosMiddlewareProvider toRosProvider(nullptr);

    // Obtain the associated klepsydra publisher
    kpsr::Publisher<std::string> * kpsrPublisher = toRosProvider.getToMiddlewareChannel<std::string, std_msgs::String>("kpsr_ros_core_test_topic", 1, nullptr, stringPublisher);
@endcode
 *
 */
class ToRosMiddlewareProvider {
public:

    /**
     * @brief ToRosMiddlewareProvider
     * @param container
     */
    ToRosMiddlewareProvider(Container * container)
        : _container(container)
    {}

    template <class T, class M>
    /**
     * @brief getToMiddlewareChannel
     * @param topic
     * @param poolSize
     * @param initializerFunction
     * @param publisher
     * @return
     */
    Publisher<T> * getToMiddlewareChannel(std::string topic, int poolSize, std::function<void(M &)> initializerFunction, ros::Publisher & publisher) {
        auto search = _publisherMap.find(topic);
        if (search != _publisherMap.end()) {
            std::shared_ptr<void> internalPointer = search->second;
            std::shared_ptr<Publisher<T>> kpsrPublisher = std::static_pointer_cast<Publisher<T>>(internalPointer);
            return kpsrPublisher.get();
        }
        else {
            ToRosChannel<M> * toRosChannel = new ToRosChannel<M>(_container, topic, poolSize, initializerFunction, publisher);
            std::shared_ptr<Publisher<T>> kpsrPublisher(new ToMiddlewareChannel<T, M>(_container, topic + "_rosstg", toRosChannel));
            std::shared_ptr<void> internalPointer = std::static_pointer_cast<void>(kpsrPublisher);
            _publisherMap[topic] = internalPointer;
            return kpsrPublisher.get();
        }
    }

    template <class T>
    /**
     * @brief getToMiddlewareChannel
     * @param topic
     * @param poolSize
     * @param initializerFunction
     * @param publisher
     * @return
     */
    Publisher<T> * getToMiddlewareChannel(std::string topic, int poolSize, std::function<void(T &)> initializerFunction, ros::Publisher & publisher) {
        auto search = _publisherMap.find(topic);
        if (search != _publisherMap.end()) {
            std::shared_ptr<void> internalPointer = search->second;
            std::shared_ptr<Publisher<T>> kpsrPublisher = std::static_pointer_cast<Publisher<T>>(internalPointer);
            return kpsrPublisher.get();
        }
        else {
            auto toRosChannel = std::make_shared<ToRosChannel<T> >(_container, topic, poolSize, initializerFunction, publisher);
            // std::shared_ptr<Publisher<T>> kpsrPublisher(new ToMiddlewareChannel<T, M>(_container, topic + "_rosstg", toRosChannel));
            std::shared_ptr<void> internalPointer = std::static_pointer_cast<void>(toRosChannel);
            _publisherMap[topic] = internalPointer;
            return toRosChannel.get();
        }
    }
private:
    Container * _container;
    std::map<std::string, std::shared_ptr<void>> _publisherMap;
};
}
}


#endif // TO_ROS_MIDDLEWARE_PROVIDER_H

