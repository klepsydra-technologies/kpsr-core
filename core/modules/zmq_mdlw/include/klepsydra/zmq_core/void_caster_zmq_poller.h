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

#ifndef VOID_CASTER_ZMQ_POLLER_H
#define VOID_CASTER_ZMQ_POLLER_H

#include <zmq.hpp>

#include <klepsydra/zmq_core/zhelpers.hpp>
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
                std::string topic = s_recv (this->_subscriber);
                zmq::message_t content;
                this->_subscriber.recv(&content);

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
