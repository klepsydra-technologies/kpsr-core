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

#ifndef BASE_SOCKET_H
#define BASE_SOCKET_H

namespace kpsr
{
namespace socket_mdlw
{
/**
 * @brief The BaseSocket class
 *
 * @copyright Klepsydra Technologies 2019-2020.
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
    int     socketId;
protected:
    static constexpr int invalidSocketId      = -1;

    // Designed to be a base class not used used directly.
    BaseSocket(int socketId);
    int getSocketId() const {return socketId;}
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
    BaseSocket(BaseSocket&& move)               noexcept;

    /**
     * @brief operator =
     * @param move
     * @return
     */
    BaseSocket& operator=(BaseSocket&& move)    noexcept;

    /**
     * @brief swap
     * @param other
     */
    void swap(BaseSocket& other)                noexcept;

    /**
     * @brief BaseSocket
     */
    BaseSocket(BaseSocket const&)               = delete;

    /**
     * @brief operator =
     * @return
     */
    BaseSocket& operator=(BaseSocket const&)    = delete;

    /**
     * @brief close
     * @details User can manually call close
     */
    void close();
};

}
}

#endif

