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

#ifndef ZMQ_BINARY_POLLER_H
#define ZMQ_BINARY_POLLER_H

#include <streambuf>

#include <zmq.hpp>

#include <klepsydra/zmq_core/non_copying_stream_buffer.h>
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
                zmq::message_t topicMsg;
                _subscriber.recv(topicMsg);
                std::string topic(static_cast<char*>(topicMsg.data()), topicMsg.size());
                zmq::message_t content;
                _subscriber.recv(content);
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
