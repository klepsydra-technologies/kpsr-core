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

#ifndef ZMQ_BINARY_POLLER_H
#define ZMQ_BINARY_POLLER_H

#include <streambuf>

#include <zmq.hpp>

#include <klepsydra/zmq_core/non_copying_stream_buffer.h>
#include <klepsydra/zmq_core/zhelpers.hpp>
#include <klepsydra/zmq_core/zmq_poller.h>

using Base = std::basic_streambuf<char> *;

namespace kpsr {
namespace zmq_mdlw {
/**
 * @brief The BinaryZMQPoller class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-zmq-internal
 */
class BinaryZMQPoller : public ZMQPoller<Base>
{
public:
    /**
     * @brief BinaryZMQPoller
     * @param subscriber
     * @param pollPeriod
     */
    BinaryZMQPoller(zmq::socket_t & subscriber, long pollPeriod)
        : ZMQPoller(subscriber, pollPeriod)
    {}

    /**
     * @brief poll
     */
    virtual void poll() {
        while (_running) {
            zmq::pollitem_t items [] = {
                { _subscriber, 0, ZMQ_POLLIN, 0 }
            };
            if (zmq::poll(items, 1, _pollPeriod) == -1)
                break;

            if (items[0].revents & ZMQ_POLLIN) {
                std::string topic = s_recv (_subscriber);
                zmq::message_t content;
                _subscriber.recv(&content);
                NonCopyingStringBuffer buffer((char *) content.data(), content.size());
                Base bufferPointer = &buffer;
                executeFunction(topic, bufferPointer);
            }
        }
    }
};
}
}

#endif // ZMQ_BINARY_POLLER_H
