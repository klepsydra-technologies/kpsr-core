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

#ifndef JSON_TO_ZMQ_CHANNEL_H
#define JSON_TO_ZMQ_CHANNEL_H

#include <zmq.hpp>

#include <exception>
#include <string>

#include <klepsydra/core/object_pool_publisher.h>

#include <klepsydra/zmq_core/zhelpers.hpp>

namespace kpsr
{
namespace zmq_mdlw
{
/**
 * @brief The JsonToZMQChannel class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-zmq-internal
 *
 * @details Klepsydra Event to ZMQ Object adapter or channel.
 *
 */
class JsonToZMQChannel : public ObjectPoolPublisher<std::string>
{

public:
    /**
     * @brief JsonToZMQChannel
     * @param environment
     * @param publisher this ZMQ socket has to be provided. Meaning that we give control to the user on how to setup ZMQ.
     * @param topic to include in the multi-part message
     */
    JsonToZMQChannel(Container * container,
                     std::string topic,
                     int poolSize,
                     std::function<void(std::string &)> initializerFunction,
                     zmq::socket_t & publisher)
        : ObjectPoolPublisher<std::string>(container, topic, "ZMQ_JSON", poolSize, initializerFunction, nullptr)
        , _publisher(publisher)
        , _topic(topic) {
    }

protected:
    void internalPublish(std::shared_ptr<const std::string> event) override {
        s_sendmore (_publisher, _topic.c_str());
        s_send (_publisher, event->c_str());
    }

private:
    zmq::socket_t & _publisher;
    std::string _topic;
};
}
}

#endif // JSON_TO_ZMQ_CHANNEL_H
