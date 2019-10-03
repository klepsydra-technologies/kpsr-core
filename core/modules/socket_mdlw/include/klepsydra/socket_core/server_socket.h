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

#ifndef SERVER_SOCKET_H
#define SERVER_SOCKET_H

#include <memory>

#include <klepsydra/socket_core/base_socket.h>
#include <klepsydra/socket_core/data_socket.h>

namespace kpsr
{
namespace socket_mdlw
{
/**
 * @brief The ServerSocket class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-sockets-internal
 *
 * @details A server socket that listens on a port for a connection
 */
class ServerSocket: public BaseSocket
{
    static constexpr int maxConnectionBacklog = 5;
public:
    /**
     * @brief ServerSocket
     * @param port
     */
    ServerSocket(int port);

    /**
     * @brief ServerSocket
     * @param socketPath
     */
    ServerSocket(std::string socketPath);

    /**
     * @brief accept
     * @param timeout_us
     * @return
     *
     * @details An accepts waits for a connection and returns a
     * socket object that can be used by the client for communication
     */
    std::shared_ptr<DataSocket> accept(int timeout_us=1000000);
};

}
}

#endif

