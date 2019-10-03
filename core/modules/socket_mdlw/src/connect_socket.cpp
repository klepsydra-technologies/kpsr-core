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
#include <cstring>

#include <iostream>

#include <klepsydra/socket_core/connect_socket.h>
#include <klepsydra/socket_core/utility.h>

kpsr::socket_mdlw::ConnectSocket::ConnectSocket(std::string const& host, int port)
    : kpsr::socket_mdlw::DataSocket(::socket(PF_INET, SOCK_STREAM, 0))
{
    struct sockaddr_in serverAddr{};
    serverAddr.sin_family       = AF_INET;
    serverAddr.sin_port         = htons(port);
    serverAddr.sin_addr.s_addr  = inet_addr(host.c_str());

    if (::connect(getSocketId(), (struct sockaddr*)&serverAddr, sizeof(serverAddr)) != 0)
    {
        close();
        throw std::runtime_error(buildErrorMessage("ConnectSocket::", __func__, ": connect: ", strerror(errno)));
    }
}

kpsr::socket_mdlw::ConnectSocket::ConnectSocket(std::string path)
    : kpsr::socket_mdlw::DataSocket(::socket(AF_UNIX, SOCK_STREAM, 0))
{
    struct sockaddr_un serverAddr{};
    serverAddr.sun_family = AF_UNIX;
    strncpy(serverAddr.sun_path, path.data(), sizeof(serverAddr.sun_path) - 1);

    if (::connect(getSocketId(), (struct sockaddr*)&serverAddr, sizeof(serverAddr)) != 0)
    {
        close();
        throw std::runtime_error(buildErrorMessage("ConnectSocket::", __func__, ": connect: ", strerror(errno)));
    }
}
