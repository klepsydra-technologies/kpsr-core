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

#ifndef VOID_CASTER_TO_ZMQ_CHANNEL_H
#define VOID_CASTER_TO_ZMQ_CHANNEL_H

#include <zmq.hpp>

#include <klepsydra/core/object_pool_publisher.h>

#include <klepsydra/zmq_core/zhelpers.hpp>

namespace kpsr
{
namespace zmq_mdlw
{
/**
 * @brief The VoidCasterToZMQChannel class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-zmq-internal
 *
 */
class VoidCasterToZMQChannel : public ObjectPoolPublisher<std::vector<unsigned char>>
{

public:
    /**
     * @brief VoidCasterToZMQChannel
     * @param environment
     * @param publisher this ZMQ socket has to be provided. Meaning that we give control to the user on how to setup ZMQ.
     * @param topic to include in the multi-part message
     */
    VoidCasterToZMQChannel(Container * container,
                       std::string topic,
                       int poolSize,
                       std::function<void(std::vector<unsigned char> &)> initializerFunction,
                       zmq::socket_t & publisher)
        : ObjectPoolPublisher<std::vector<unsigned char>>(container, topic, "ZMQ_VOID_CASTER", std::max(1, poolSize), initializerFunction, nullptr)
        , _publisher(publisher)
        , _topic(topic)
    {}

protected:
    void internalPublish(std::shared_ptr<const std::vector<unsigned char>> event) override {
        s_sendmore (_publisher, _topic.c_str());
        zmq::message_t message(event->data(), event->size());
        _publisher.send(message);
    }

private:
    zmq::socket_t & _publisher;
    std::string _topic;
};
}
}

#endif // VOID_CASTER_TO_ZMQ_CHANNEL_H
