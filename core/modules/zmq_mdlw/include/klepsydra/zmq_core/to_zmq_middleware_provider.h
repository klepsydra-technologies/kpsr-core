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

#ifndef TO_ZMQ_MIDDLEWARE_PROVIDER_H
#define TO_ZMQ_MIDDLEWARE_PROVIDER_H

#include <map>
#include <memory>

#include <klepsydra/core/to_middleware_channel.h>

#include <klepsydra/zmq_core/binary_to_zmq_channel.h>
#include <klepsydra/zmq_core/json_to_zmq_channel.h>
#include <klepsydra/zmq_core/void_caster_to_zmq_channel.h>

namespace kpsr {
namespace zmq_mdlw {
/**
 * @brief The ToZMQMiddlewareProvider class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-zmq-composition
 *
 * @details Klepsydra Event to ZMQ Object adapter using Cereal binary serialization. Similarly to other middleware
 * implementations, this class offers the posibility to create a pool of binary serialization object in order
 * to improve performance. This turns out to be quite natural combination with ZMQ. The following example
 * ilustrates the use of this class:
@code
    // configure zmq socket to publish
    std::string serverUrl = "tcp://*:5556";
    std::string topic = "Weather";

    zmq::context_t context (1);
    zmq::socket_t publisher (context, ZMQ_PUB);
    publisher.bind(serverUrl);
    publisher.bind("ipc://weather.ipc");

    // create provider. One per socket
    kpsr::zmq_mdlw::ToZMQMiddlewareProvider toZMQMiddlewareProvider(nullptr, publisher);

    // get the instance of the publisher.
    kpsr::Publisher<WeatherData> * toZMQPublisher = toZMQMiddlewareProvider.getBinaryToMiddlewareChannel<WeatherData>(topic, 0);
@endcode
 */
class ToZMQMiddlewareProvider {
public:
    /**
     * @brief ToZMQMiddlewareProvider
     * @param container
     * @param zmqPublisher
     */
    ToZMQMiddlewareProvider(Container * container, zmq::socket_t & zmqPublisher)
        : _container(container)
        , _zmqPublisher(zmqPublisher)
    {}

    template <class T>
    /**
     * @brief getBinaryToMiddlewareChannel
     * @param topic zmq topic
     * @param poolSize object pool size. 0 for no object pool
     * @return binary serializer klepsydra to zmq publisher
     */
    Publisher<T> * getBinaryToMiddlewareChannel(const std::string & topic, int poolSize = 0) {
        auto search = _binaryPublisherMap.find(topic);
        if (search != _binaryPublisherMap.end()) {
            std::shared_ptr<void> internalPointer = search->second;
            std::shared_ptr<Publisher<T>> publisher = std::static_pointer_cast<Publisher<T>>(internalPointer);
            return publisher.get();
        }
        else {
            std::function<void(Base &)> _initializerFunction = [](Base & event) {
                                                                   if (!event) {
                                                                       event = new std::stringbuf;
                                                                   }
                                                                   event->pubseekpos(0);
                                                               };
            BinaryToZMQChannel * toZmqChannel = new BinaryToZMQChannel(_container, topic, poolSize, _initializerFunction, _zmqPublisher);
            std::shared_ptr<Publisher<T>> publisher(new ToMiddlewareChannel<T, Base>(_container, topic + "_zmq", toZmqChannel));
            std::shared_ptr<void> internalPointer = std::static_pointer_cast<void>(publisher);
            _binaryPublisherMap[topic] = internalPointer;
            return publisher.get();
        }
    }

    template <class T>
    /**
     * @brief getJsonToMiddlewareChannel
     * @param topic zmq topic
     * @param poolSize object pool size. 0 for no object pool
     * @return json serializer klepsydra to zmq publisher
     */
    Publisher<T> * getJsonToMiddlewareChannel(const std::string & topic, int poolSize = 0) {
        auto search = _jsonPublisherMap.find(topic);
        if (search != _jsonPublisherMap.end()) {
            std::shared_ptr<void> internalPointer = search->second;
            std::shared_ptr<Publisher<T>> publisher = std::static_pointer_cast<Publisher<T>>(internalPointer);
            return publisher.get();
        }
        else {
            JsonToZMQChannel * toZmqChannel = new JsonToZMQChannel(_container, topic, poolSize, nullptr, _zmqPublisher);
            std::shared_ptr<Publisher<T>> publisher(new ToMiddlewareChannel<T, std::string>(_container, topic + "_zmq", toZmqChannel));
            std::shared_ptr<void> internalPointer = std::static_pointer_cast<void>(publisher);
            _jsonPublisherMap[topic] = internalPointer;
            return publisher.get();
        }
    }

    template <class T>
    /**
     * @brief getVoidCasterToMiddlewareChannel
     * @param topic zmq topic
     * @param poolSize object pool size. 0 for no object pool
     * @return non serializer klepsydra to zmq publisher
     */
    Publisher<T> * getVoidCasterToMiddlewareChannel(const std::string & topic, int poolSize = 0) {
        auto search = _voidCasterPublisherMap.find(topic);
        if (search != _voidCasterPublisherMap.end()) {
            std::shared_ptr<void> internalPointer = search->second;
            std::shared_ptr<Publisher<T>> publisher = std::static_pointer_cast<Publisher<T>>(internalPointer);
            return publisher.get();
        }
        else {
            VoidCasterToZMQChannel * toZmqChannel = new VoidCasterToZMQChannel(_container, topic, poolSize, nullptr, _zmqPublisher);
            std::shared_ptr<Publisher<T>> publisher(new ToMiddlewareChannel<T, std::vector<unsigned char>>(_container, topic + "_zmq", toZmqChannel));
            std::shared_ptr<void> internalPointer = std::static_pointer_cast<void>(publisher);
            _voidCasterPublisherMap[topic] = internalPointer;
            return publisher.get();
        }
    }

private:
    Container * _container;
    zmq::socket_t & _zmqPublisher;
    std::map<std::string, std::shared_ptr<void>> _binaryPublisherMap;
    std::map<std::string, std::shared_ptr<void>> _jsonPublisherMap;
    std::map<std::string, std::shared_ptr<void>> _voidCasterPublisherMap;
};
}
}

#endif // TO_ZMQ_MIDDLEWARE_PROVIDER_H
