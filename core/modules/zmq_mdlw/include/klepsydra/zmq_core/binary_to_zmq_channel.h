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

#ifndef BINARY_TO_ZMQ_CHANNEL_H
#define BINARY_TO_ZMQ_CHANNEL_H

#include <zmq.hpp>

#include <memory>
#include <streambuf>
#include <string>

#include <klepsydra/core/object_pool_publisher.h>

using Base = std::basic_streambuf<char> *;

namespace kpsr {
namespace zmq_mdlw {
/**
 * @brief The BinaryToZMQChannel class
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-zmq-internal
 *
 */
class BinaryToZMQChannel : public ObjectPoolPublisher<Base>
{
public:
    /**
     * @brief BinaryToZMQChannel
     * @param environment
     * @param publisher this ZMQ socket has to be provided. Meaning that we give control to the user on how to setup ZMQ.
     * @param topic to include in the multi-part message
     */
    BinaryToZMQChannel(Container *container,
                       std::string topic,
                       int poolSize,
                       std::function<void(Base &)> initializerFunction,
                       zmq::socket_t &publisher)
        : ObjectPoolPublisher<Base>(container,
                                    topic,
                                    "ZMQ_BINARY",
                                    std::max(1, poolSize),
                                    initializerFunction,
                                    nullptr)
        , _publisher(publisher)
        , _topic(topic)
    {}

protected:
    void internalPublish(std::shared_ptr<const Base> event) override
    {
        std::stringbuf *buffer = (std::stringbuf *) (*event);
        _publisher.send(zmq::const_buffer(_topic.c_str(), _topic.size()), zmq::send_flags::sndmore);
        zmq::message_t message(buffer->str().data(), buffer->str().size());
        _publisher.send(message);
    }

private:
    zmq::socket_t &_publisher;
    std::string _topic;
};
} // namespace zmq_mdlw
} // namespace kpsr

#endif // BINARY_TO_ZMQ_CHANNEL_H
