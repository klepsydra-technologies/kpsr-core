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

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <memory>
#include <string>

#include <klepsydra/socket_core/data_socket.h>

namespace kpsr {
namespace socket_mdlw {
/**
 * @brief The Protocol class
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-sockets-internal
 *
 */
class Protocol
{
protected:
    std::shared_ptr<DataSocket> _dataSocket;

public:
    /**
     * @brief Protocol
     * @param dataSocket
     */
    Protocol(std::shared_ptr<DataSocket> dataSocket);

    ~Protocol();

    /**
     * @brief sendMessage
     * @param message
     */
    virtual void sendMessage(const std::string &message) = 0;

    /**
     * @brief recvMessage
     * @param message
     */
    virtual void recvMessage(std::string &message) = 0;
};

} // namespace socket_mdlw
} // namespace kpsr

#endif
