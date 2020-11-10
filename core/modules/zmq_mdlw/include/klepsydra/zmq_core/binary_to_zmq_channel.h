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

#ifndef BINARY_TO_ZMQ_CHANNEL_H
#define BINARY_TO_ZMQ_CHANNEL_H

#include <zmq.hpp>

#include <memory>
#include <streambuf>
#include <string>

#include <klepsydra/core/object_pool_publisher.h>

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
        _publisher.send(zmq::const_buffer(_topic.c_str(), _topic.size()), zmq::send_flags::sndmore);
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
