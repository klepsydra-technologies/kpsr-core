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

#ifndef FROM_ZMQ_MIDDLEWARE_PROVIDER_H
#define FROM_ZMQ_MIDDLEWARE_PROVIDER_H

#include <map>
#include <memory>

#include <spdlog/spdlog.h>


#include <zmq.hpp>

#include <klepsydra/core/from_middleware_channel.h>

#include <klepsydra/zmq_core/binary_zmq_poller.h>
#include <klepsydra/zmq_core/json_zmq_poller.h>
#include <klepsydra/zmq_core/void_caster_zmq_poller.h>

namespace kpsr
{
namespace zmq_mdlw
{

/**
 * @brief The FromZmqChannel class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-zmq-composition
 *
 * @details ZMQ Message to Klepsydra Event channel. It works similarly to other middleware implementations,
 * It has two variations: binary and json deserialization, which is decided via injection.
 *
 */
template<class U>
class FromZmqChannel
{
public:
    /**
     * @brief FromZmqChannel
     * @param zmqPoller injected object that establish the type of deserialization
     */
    FromZmqChannel(ZMQPoller<U> * zmqPoller)
        : _zmqPoller(zmqPoller)
    {}

    template<class T>
    /**
     * @brief registerToTopic
     * @param topic zmq topic to listen to
     * @param internalPublisher Klepsydra publisher to send the deserialized event to.
     */
    void registerToTopic(const std::string & topic, Publisher<T> * internalPublisher) {
        auto search = _subscriberMap.find(topic);
        if (search == _subscriberMap.end()) {
            std::shared_ptr<FromMiddlewareChannel<T, U>> fromMiddlewareChannel(new FromMiddlewareChannel<T, U>(internalPublisher));
            std::function<void(U)> onDataAvailableFunction = std::bind(&FromMiddlewareChannel<T, U>::onMiddlewareMessage, fromMiddlewareChannel.get(), std::placeholders::_1);
            _zmqPoller->registerToTopic(topic, onDataAvailableFunction);

            std::shared_ptr<void> internalPointer = std::static_pointer_cast<void>(fromMiddlewareChannel);
            _subscriberMap[topic] = internalPointer;
        }
    }

    /**
     * @brief unregisterFromTopic
     * @param topic
     */
    void unregisterFromTopic(const std::string & topic) {
        _zmqPoller->unregisterFromTopic(topic);
    }

    /**
     * @brief start Launches the polling thread.
     */
    void start() {
        _zmqPoller->start();
    }

    /**
     * @brief stop
     */
    void stop() {
        _zmqPoller->stop();
    }

private:
    ZMQPoller<U> * _zmqPoller;

    std::map<std::string, std::shared_ptr<void>> _subscriberMap;
};

/**
 * @brief The FromZmqMiddlewareProvider class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-zmq-composition
 *
 * @details This class is a wizard to create from zmq channel objects. It simplifies all the wiring for deserialization
 * and instatiation of objects. The following examples ilustrates this:
@code
    //  Example of ZMQ socket creation
    zmq::context_t context (1);
    spdlog::info("Collecting updates from weather server...\n");
    zmq::socket_t subscriber (context, ZMQ_SUB);

    subscriber.connect(url);
    subscriber.setsockopt(ZMQ_SUBSCRIBE, topic.c_str(), topic.size());

    // Create an instance FromZmqMiddlewareProvider. Once per application.
    kpsr::zmq_mdlw::FromZmqMiddlewareProvider _fromZmqMiddlewareProvider;
    _jsomFromZMQProvider->start();

    // Create a fromChannel for the above created zmq socket.
    kpsr::zmq_mdlw::FromZmqChannel<std::string> * _jsomFromZMQProvider = _fromZmqMiddlewareProvider.getJsonFromMiddlewareChannel<WeatherData>(subscriber, 100);

    // Create klepsydra publisher subscriber.
    kpsr::mem::SafeQueueMiddlewareProvider<WeatherData> _safeQueueProvider(nullptr, "weatherData", 4, 6, nullptr, nullptr, true);
    _safeQueueProvider.start();

    // Register the topic. Now the aubscriber in the above created pub/sub pair will start receiving events.
    _jsomFromZMQProvider->registerToTopic(topic, _safeQueueProvider.getPublisher());
@endcode
 *
 */
class FromZmqMiddlewareProvider {

public:
    template<class U>
    /**
     * @brief getBinaryFromMiddlewareChannel
     * @param subscriber ZMQ listeing socket
     * @param pollPeriod in milliseconds
     * @return a FromZmqChannel with binary deserialization
     */
    FromZmqChannel<Base> * getBinaryFromMiddlewareChannel(zmq::socket_t & subscriber, long pollPeriod) {
        BinaryZMQPoller * binaryZMQPoller = new BinaryZMQPoller(subscriber, pollPeriod);
        return new FromZmqChannel<Base>(binaryZMQPoller);
    }

    template<class U>
    /**
     * @brief getJsonFromMiddlewareChannel
     * @param subscriber ZMQ listeing socket
     * @param pollPeriod in milliseconds
     * @return a FromZmqChannel with json deserialization
     */
    FromZmqChannel<std::string> * getJsonFromMiddlewareChannel(zmq::socket_t & subscriber, long pollPeriod) {
        JsonZMQPoller * jsonZMQPoller = new JsonZMQPoller(subscriber, pollPeriod);
        return new FromZmqChannel<std::string>(jsonZMQPoller);
    }

    template<class U>
    /**
     * @brief getJsonFromMiddlewareChannel
     * @param subscriber ZMQ listeing socket
     * @param pollPeriod in milliseconds
     * @return a FromZmqChannel with json deserialization
     */
    FromZmqChannel<std::vector<unsigned char>> * getVoidCasterFromMiddlewareChannel(zmq::socket_t & subscriber, long pollPeriod) {
        VoidCasterZMQPoller * voidCasterZMQPoller = new VoidCasterZMQPoller(subscriber, pollPeriod);
        return new FromZmqChannel<std::vector<unsigned char>>(voidCasterZMQPoller);
    }
};

}
}
#endif // FROM_ZMQ_MIDDLEWARE_PROVIDER_H
