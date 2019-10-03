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

#ifndef BINARY_TO_ZMQ_CHANNEL_H
#define BINARY_TO_ZMQ_CHANNEL_H

#include <zmq.hpp>

#include <memory>
#include <streambuf>
#include <string>

#include <klepsydra/core/object_pool_publisher.h>

#include <klepsydra/zmq_core/zhelpers.hpp>

using Base = std::basic_streambuf<char> *;

namespace kpsr
{
namespace zmq_mdlw
{
/**
 * @brief The BinaryToZMQChannel class
 *
 * @copyright Klepsydra Technologies 2019-2020.
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
    BinaryToZMQChannel(Container * container,
                       std::string topic,
                       int poolSize,
                       std::function<void(Base &)> initializerFunction,
                       zmq::socket_t & publisher)
        : ObjectPoolPublisher<Base>(container, topic, "ZMQ_BINARY", std::max(1, poolSize), initializerFunction, nullptr)
        , _publisher(publisher)
        , _topic(topic)
    {}

protected:

    void internalPublish(std::shared_ptr<const Base> event) override {
        std::stringbuf * buffer = (std::stringbuf *) (* event);
        s_sendmore (_publisher, _topic.c_str());
        zmq::message_t message(buffer->str().data(), buffer->str().size());
        _publisher.send(message);
    }

private:
    zmq::socket_t & _publisher;
    std::string _topic;
};
}
}

#endif // BINARY_TO_ZMQ_CHANNEL_H
