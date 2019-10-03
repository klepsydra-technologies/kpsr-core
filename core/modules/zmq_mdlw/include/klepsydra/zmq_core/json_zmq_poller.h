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

#ifndef ZMQ_JSON_POLLER_H
#define ZMQ_JSON_POLLER_H

#include <zmq.hpp>
#include <iostream>

#include <klepsydra/zmq_core/zhelpers.hpp>
#include <klepsydra/zmq_core/zmq_poller.h>

namespace kpsr {
namespace zmq_mdlw {
/**
 * @brief The JsonZMQPoller class
 *
 * @copyright Klepsydra Technologies 2019-2020.
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
    JsonZMQPoller(zmq::socket_t & subscriber, long pollPeriod)
        : ZMQPoller(subscriber, pollPeriod)
    {}

    /**
     * @brief poll
     */
    void poll() {
        while (_running) {
            zmq::pollitem_t items [] = {
                { _subscriber, 0, ZMQ_POLLIN, 0 }
            };
            if (zmq::poll(items, 1, _pollPeriod) == -1)
                break;

            if (items[0].revents & ZMQ_POLLIN) {
                std::string topic = s_recv (_subscriber);
                std::string contents = s_recv (_subscriber);
                executeFunction(topic, contents);
            }
        }
    }
};
}
}

#endif // ZMQ_JSON_POLLER_H
