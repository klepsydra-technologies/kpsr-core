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

#ifndef DATA_SOCKET_H
#define DATA_SOCKET_H

#include <klepsydra/socket_core/base_socket.h>

namespace kpsr {
namespace socket_mdlw {
/**
 * @brief The DataSocket class
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-sockets-internal
 *
 * @details A class that can read/write to a socket
 */
class DataSocket : public BaseSocket
{
public:
    /**
     * @brief DataSocket
     * @param socketId
     */
    explicit DataSocket(int socketId)
        : BaseSocket(socketId)
    {}

    template<typename F>
    /**
     * @brief getMessageData
     * @param buffer
     * @param size
     * @param scanForEnd
     */
    std::size_t getMessageData(
        char *buffer, std::size_t size, F scanForEnd = [](std::size_t) { return false; });

    /**
     * @brief putMessageData
     * @param buffer
     * @param size
     */
    void putMessageData(char const *buffer, std::size_t size);

    /**
     * @brief putMessageClose
     */
    void putMessageClose();
};

} // namespace socket_mdlw
} // namespace kpsr

#include <klepsydra/socket_core/data_socket.tpp>

#endif
