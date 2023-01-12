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

#ifndef BASE_SOCKET_H
#define BASE_SOCKET_H

namespace kpsr {
namespace socket_mdlw {
/**
 * @brief The BaseSocket class
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-sockets-internal
 *
 * @details A RAII base class for handling sockets. Socket is movable but not copyable.
 *
 */
class BaseSocket
{
    /**
     * @brief socketId
     */
    int socketId;

protected:
    /**
     * @brief invalidSocketId
     */
    static constexpr int invalidSocketId = -1;

    /**
     * @brief Constructor
     *
     * Designed to be a base class not used used directly.
     */
    explicit BaseSocket(int socketId);

    /**
     * @brief getSocketId
     */
    int getSocketId() const { return socketId; }

public:
    /**
     * @brief ~BaseSocket
     */
    virtual ~BaseSocket();

    /**
     * @brief BaseSocket
     * @param move
     * @details Moveable but not Copyable
     */
    BaseSocket(BaseSocket &&move) noexcept;

    /**
     * @brief operator =
     * @param move
     * @return
     */
    BaseSocket &operator=(BaseSocket &&move) noexcept;

    /**
     * @brief swap
     * @param other
     */
    void swap(BaseSocket &other) noexcept;

    /**
     * @brief BaseSocket
     */
    BaseSocket(BaseSocket const &) = delete;

    /**
     * @brief operator =
     * @return
     */
    BaseSocket &operator=(BaseSocket const &) = delete;

    /**
     * @brief close
     * @details User can manually call close
     */
    void close();
};

} // namespace socket_mdlw
} // namespace kpsr

#endif
