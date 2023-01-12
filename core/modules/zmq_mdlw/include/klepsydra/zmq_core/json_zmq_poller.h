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

#ifndef ZMQ_JSON_POLLER_H
#define ZMQ_JSON_POLLER_H

#include <iostream>
#include <zmq.hpp>

#include <klepsydra/zmq_core/zmq_poller.h>

namespace kpsr {
namespace zmq_mdlw {
/**
 * @brief The JsonZMQPoller class
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-zmq-internal
 *
 */
class JsonZMQPoller : public ZMQPoller<std::string>
{
public:
    /**
     * @brief JsonZMQPoller
     * @param subscriber
     * @param pollPeriod
     */
    JsonZMQPoller(zmq::socket_t &subscriber, long pollPeriod)
        : ZMQPoller(subscriber, pollPeriod)
    {}

    /**
     * @brief poll
     */
    void poll()
    {
        while (_running) {
            zmq::pollitem_t items[] = {{_subscriber, 0, ZMQ_POLLIN, 0}};
            if (zmq::poll(items, 1, _pollPeriod) == -1)
                break;

            if (items[0].revents & ZMQ_POLLIN) {
                zmq::message_t topicMsg;
                zmq::message_t content;
                _subscriber.recv(topicMsg);
                _subscriber.recv(content);
                std::string topic(static_cast<char *>(topicMsg.data()), topicMsg.size());
                std::string contentString(static_cast<char *>(content.data()), content.size());
                executeFunction(topic, contentString);
            }
        }
    }
};
} // namespace zmq_mdlw
} // namespace kpsr

#endif // ZMQ_JSON_POLLER_H
