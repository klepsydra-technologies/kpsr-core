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

#ifndef FROM_ROS_MIDDLEWARE_PROVIDER_H
#define FROM_ROS_MIDDLEWARE_PROVIDER_H

#include <map>
#include <memory>

#include "ros/ros.h"

#include <klepsydra/core/event_emitter_subscriber.h>
#include <klepsydra/core/from_middleware_channel.h>

namespace kpsr
{
namespace ros_mdlw
{
template<class T, class M>
/**
 * @brief The RosSubscriptionData struct
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version 2.0.1
 *
 * @ingroup kpsr-rosstg-internal
 *
 */
struct RosSubscriptionData {
public:
    /**
    * @brief RosSubscriptionData
    * @param rosNode
    * @param topicName
    * @param queueSize
    * @param internalPublisher
    */
    RosSubscriptionData(ros::NodeHandle & rosNode, const char * topicName, int queueSize, Publisher<T> * internalPublisher)
        : _fromMiddlewareChannel(internalPublisher) {
        _rosSubscriber = rosNode.subscribe(topicName, queueSize, &FromMiddlewareChannel<T, M>::onMiddlewareMessage, &_fromMiddlewareChannel);
    }

private:
    FromMiddlewareChannel<T, M> _fromMiddlewareChannel;
    ros::Subscriber _rosSubscriber;
};

/**
 * @brief The FromRosMiddlewareProvider class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version 2.0.1
 *
 * @ingroup kpsr-rosstg-composition
 *
 * @details ROS middleware to Klepsydra adapter or channel. It extends the event_emitter_subscriber.h
 * class in order to keep track of klepsydra listeners. Its use is very straightforward as per the following
 * example:
 *
@code
    // Initialize ROS
    ros::init(argc, argv, "kpsr_ros_core_test");
    ros::NodeHandle nodeHandle;
    ros::Rate rate(100);

    // Create a from ros provider Klepsydra wizard instance. Once for the whole application.
    kpsr::ros_mdlw::FromRosMiddlewareProvider fromRosProvider(nodeHandle);

    // Create a Klepsydra pub/sub pair
    kpsr::mem::SafeQueueMiddlewareProvider<std::string> safeQueueProvider(nullptr, "test", 8, 0, nullptr, nullptr, false);
    safeQueueProvider.start();

    // Obtain a from ros channel and provide the klepsydra publisher
    fromRosProvider.registerToTopic<std::string, std_msgs::String>("kpsr_ros_core_test_topic", 1, safeQueueProvider.getPublisher());

    // Now, listeners can be registered to the corresponding subscriber.
    kpsr::mem::CacheListener<std::string> cacheListener;
    safeQueueProvider.getSubscriber()->registerListener("cacheListener", cacheListener.cacheListenerFunction);
@endcode
 *
 */
class FromRosMiddlewareProvider {
public:
    /**
     * @brief FromRosMiddlewareProvider
     * @param rosNode
     */
    FromRosMiddlewareProvider(ros::NodeHandle & rosNode)
        : _rosNode(rosNode)
    {}

    template<class T, class M>
    /**
     * @brief registerToTopic
     * @param topicName
     * @param queueSize
     * @param internalPublisher
     */
    void registerToTopic(const char * topicName, int queueSize, Publisher<T> * internalPublisher) {
        auto search = _subscriberMap.find(topicName);
        if (search == _subscriberMap.end()) {
            std::shared_ptr<RosSubscriptionData<T, M>> rosSubscriptionData = std::shared_ptr<RosSubscriptionData<T, M>>(new RosSubscriptionData<T, M>(_rosNode, topicName, queueSize, internalPublisher));
            std::shared_ptr<void> internalPointer = std::static_pointer_cast<void>(rosSubscriptionData);
            _subscriberMap[topicName] = internalPointer;
        }
    }

    template<class T>
    /**
     * @brief registerToTopic
     * @param topicName
     * @param queueSize
     * @param internalPublisher
     */
    void registerToTopic(const char * topicName, int queueSize, Publisher<T> * internalPublisher) {
        auto search = _subscriberMap.find(topicName);
        if (search == _subscriberMap.end()) {
            auto rosSubscriptionData = std::make_shared<RosSubscriptionData<T, T>>(_rosNode, topicName, queueSize, internalPublisher);
            std::shared_ptr<void> internalPointer = std::static_pointer_cast<void>(rosSubscriptionData);
            _subscriberMap[topicName] = internalPointer;
        }
    }

private:
    ros::NodeHandle & _rosNode;
    std::map<std::string, std::shared_ptr<void>> _subscriberMap;
};
}
}
#endif
