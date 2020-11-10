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

#ifndef VOID_CASTER_ZMQ_POLLER_H
#define VOID_CASTER_ZMQ_POLLER_H

#include <zmq.hpp>

#include <klepsydra/zmq_core/zmq_poller.h>

namespace kpsr {
namespace zmq_mdlw {
/**
 * @brief The VoidCasterZMQPoller class
 *
 * @copyright Klepsydra Technologies 2019-2020.
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
    VoidCasterZMQPoller(zmq::socket_t & subscriber, long pollPeriod)
        : ZMQPoller<std::vector<unsigned char>>(subscriber, pollPeriod)
    {}

    /**
     * @brief poll
     */
    void poll() override {
        while (this->_running) {
            zmq::pollitem_t items [] = {
                { this->_subscriber, 0, ZMQ_POLLIN, 0 }
            };
            if (zmq::poll(items, 1, this->_pollPeriod) == -1)
                break;

            if (items[0].revents & ZMQ_POLLIN) {
                zmq::message_t topicMsg;
                _subscriber.recv(topicMsg);
                std::string topic(static_cast<char*>(topicMsg.data()), topicMsg.size());
                zmq::message_t content;
                this->_subscriber.recv(content);

                unsigned char* data = (unsigned char*)content.data();
                std::vector<unsigned char> event(content.size());
                memcpy(&event[0], data, content.size());

                this->executeFunction(topic, event);
            }
        }
    }
};
}
}

#endif // VOID_CASTER_ZMQ_POLLER_H
