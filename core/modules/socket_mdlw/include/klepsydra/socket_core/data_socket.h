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

#ifndef DATA_SOCKET_H
#define DATA_SOCKET_H

#include <klepsydra/socket_core/base_socket.h>

namespace kpsr
{
namespace socket_mdlw
{
/**
 * @brief The DataSocket class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-sockets-internal
 *
 * @details A class that can read/write to a socket
 */
class DataSocket: public BaseSocket
{
public:
    /**
     * @brief DataSocket
     * @param socketId
     */
    DataSocket(int socketId)
        : BaseSocket(socketId)
    {}

    template<typename F>
    /**
     * @brief getMessageData
     * @param buffer
     * @param size
     * @param scanForEnd
     */
    std::size_t getMessageData(char* buffer, std::size_t size, F scanForEnd = [](std::size_t){return false;});

    /**
     * @brief putMessageData
     * @param buffer
     * @param size
     */
    void        putMessageData(char const* buffer, std::size_t size);

    /**
     * @brief putMessageClose
     */
    void        putMessageClose();
};


}
}

#include <klepsydra/socket_core/data_socket.tpp>

#endif

