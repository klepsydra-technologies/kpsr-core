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

#ifndef CONNECT_SOCKET_H
#define CONNECT_SOCKET_H

#include <klepsydra/socket_core/data_socket.h>

namespace kpsr {
namespace socket_mdlw {

/**
 * @brief The ConnectSocket class
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-sockets-internal
 *
 * @details A class the conects to a remote machine. Allows read/write accesses to the remote machine.
 */
class ConnectSocket : public DataSocket
{
public:
    /**
     * @brief ConnectSocket
     * @param host
     * @param port
     */
    ConnectSocket(const std::string &host, int port);

    /**
     * @brief ConnectSocket
     * @param path
     */
    explicit ConnectSocket(const std::string &path);
};

} // namespace socket_mdlw
} // namespace kpsr

#endif
