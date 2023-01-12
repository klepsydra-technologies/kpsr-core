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

#ifndef VOID_CASTER_ZMQ_POLLER_H
#define VOID_CASTER_ZMQ_POLLER_H

#include <zmq.hpp>

#include <klepsydra/zmq_core/zmq_poller.h>

namespace kpsr {
namespace zmq_mdlw {
/**
 * @brief The VoidCasterZMQPoller class
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-zmq-internal
 */
class VoidCasterZMQPoller : public ZMQPoller<std::vector<unsigned char>>
{
public:
    /**
     * @brief VoidCasterZMQPoller
     * @param subscriber
     * @param pollPeriod
     */
    VoidCasterZMQPoller(zmq::socket_t &subscriber, long pollPeriod)
        : ZMQPoller<std::vector<unsigned char>>(subscriber, pollPeriod)
    {}

    /**
     * @brief poll
     */
    void poll() override
    {
        while (this->_running) {
            zmq::pollitem_t items[] = {{this->_subscriber, 0, ZMQ_POLLIN, 0}};
            if (zmq::poll(items, 1, this->_pollPeriod) == -1)
                break;

            if (items[0].revents & ZMQ_POLLIN) {
                zmq::message_t topicMsg;
                _subscriber.recv(topicMsg);
                std::string topic(static_cast<char *>(topicMsg.data()), topicMsg.size());
                zmq::message_t content;
                this->_subscriber.recv(content);

                unsigned char *data = (unsigned char *) content.data();
                std::vector<unsigned char> event(content.size());
                memcpy(&event[0], data, content.size());

                this->executeFunction(topic, event);
            }
        }
    }
};
} // namespace zmq_mdlw
} // namespace kpsr

#endif // VOID_CASTER_ZMQ_POLLER_H
