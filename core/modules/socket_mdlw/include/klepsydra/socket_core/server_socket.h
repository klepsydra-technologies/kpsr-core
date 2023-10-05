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

#ifndef SERVER_SOCKET_H
#define SERVER_SOCKET_H

#include <memory>

#include <klepsydra/socket_core/base_socket.h>
#include <klepsydra/socket_core/data_socket.h>

namespace kpsr {
namespace socket_mdlw {
/**
 * @brief The ServerSocket class
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-sockets-internal
 *
 * @details A server socket that listens on a port for a connection
 */
class ServerSocket : public BaseSocket
{
    static constexpr int maxConnectionBacklog = 5;

public:
    /**
     * @brief ServerSocket
     * @param port
     */
    explicit ServerSocket(int port);

    /**
     * @brief ServerSocket
     * @param socketPath
     */
    explicit ServerSocket(const std::string &socketPath);

    /**
     * @brief accept
     * @param timeout
     * @return
     *
     * @details An accepts waits for a connection and returns a
     * socket object that can be used by the client for communication
     */
    std::shared_ptr<DataSocket> accept(int timeout = 1);
};

} // namespace socket_mdlw
} // namespace kpsr

#endif
