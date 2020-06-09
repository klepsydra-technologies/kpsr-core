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
    Publisher<T> * getToMiddlewareChannel(const std::string & topic, int poolSize, std::function<void(M &)> initializerFunction, ros::Publisher & publisher) {
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
    Publisher<T> * getToMiddlewareChannel(const std::string & topic, int poolSize, std::function<void(T &)> initializerFunction, ros::Publisher & publisher) {
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

