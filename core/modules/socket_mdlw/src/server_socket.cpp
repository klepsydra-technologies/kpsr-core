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

#include <arpa/inet.h>
#include <sys/un.h>

#include <unistd.h>
#include <fcntl.h>

#include <klepsydra/socket_core/server_socket.h>
#include <klepsydra/socket_core/utility.h>

kpsr::socket_mdlw::ServerSocket::ServerSocket(int port)
    : BaseSocket(::socket(PF_INET, SOCK_STREAM, 0))
{
    struct sockaddr_in serverAddr;
    bzero((char*)&serverAddr, sizeof(serverAddr));
    serverAddr.sin_family       = AF_INET;
    serverAddr.sin_port         = htons(port);
    serverAddr.sin_addr.s_addr  = INADDR_ANY;

    // Set the server socket to non-blocking
    ::fcntl(getSocketId(), F_SETFL, O_NONBLOCK);

    if (::bind(getSocketId(), (struct sockaddr *) &serverAddr, sizeof(serverAddr)) != 0)
    {
        close();
        throw std::runtime_error(buildErrorMessage("ServerSocket::", __func__, ": bind: ", strerror(errno)));
    }

    if (::listen(getSocketId(), maxConnectionBacklog) != 0)
    {
        close();
        throw std::runtime_error(buildErrorMessage("ServerSocket::", __func__, ": listen: ", strerror(errno)));
    }
}

kpsr::socket_mdlw::ServerSocket::ServerSocket(std::string socketPath)
    : BaseSocket(::socket(AF_UNIX, SOCK_STREAM, 0))
{
    struct sockaddr_un serverAddr{};
    serverAddr.sun_family = AF_UNIX;
    strncpy(serverAddr.sun_path, socketPath.data(), sizeof(serverAddr.sun_path) - 1);

    // Set the server socket to non-blocking
    ::fcntl(getSocketId(), F_SETFL, O_NONBLOCK);

    if (::bind(getSocketId(), (struct sockaddr *) &serverAddr, sizeof(serverAddr)) != 0)
    {
        close();
        throw std::runtime_error(buildErrorMessage("ServerSocket::", __func__, ": bind: ", strerror(errno)));
    }

    if (::listen(getSocketId(), maxConnectionBacklog) != 0)
    {
        close();
        throw std::runtime_error(buildErrorMessage("ServerSocket::", __func__, ": listen: ", strerror(errno)));
    }
}

std::shared_ptr<kpsr::socket_mdlw::DataSocket> kpsr::socket_mdlw::ServerSocket::accept(int timeout_us)
{
    if (getSocketId() == invalidSocketId)
    {
        throw std::logic_error(buildErrorMessage("ServerSocket::", __func__, ": accept called on a bad socket object (this object was moved)"));
    }

    struct timeval tv;
    fd_set readfds;

    tv.tv_sec = 0;
    tv.tv_usec = timeout_us;

    FD_ZERO(&readfds);
    FD_SET(getSocketId(), &readfds);

    // Wait until the socket is ready to accept, or a timeout occurs
    ::select(getSocketId()+1, &readfds, NULL, NULL, &tv);

    if (!FD_ISSET(getSocketId(), &readfds)) {
        throw std::runtime_error(buildErrorMessage("ServerSocket:", __func__, ": accept: Timed out"));
    }

    struct  sockaddr_storage    serverStorage;
    socklen_t                   addr_size   = sizeof serverStorage;
    int newSocket = ::accept(getSocketId(), (struct sockaddr*)&serverStorage, &addr_size);
    if (newSocket == -1)
    {
        throw std::runtime_error(buildErrorMessage("ServerSocket:", __func__, ": accept: ", strerror(errno)));
    }
    return std::shared_ptr<kpsr::socket_mdlw::DataSocket>(new kpsr::socket_mdlw::DataSocket(newSocket));
}

