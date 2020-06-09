/****************************************************************************
*
*                           Klepsydra Core Modules
*              Copyright (C) 2019-2020  Klepsydra Technologies GmbH
*                            All Rights Reserved.
*
*  This file is subject to the terms and conditions defined in
*  file 'LICENSE.md', which is part of this source code package.
*
*  NOTICE:  All information contained herein is, and remains the property of Klepsydra
*  Technologies GmbH and its suppliers, if any. The intellectual and technical concepts
*  contained herein are proprietary to Klepsydra Technologies GmbH and its suppliers and
*  may be covered by Swiss and Foreign Patents, patents in process, and are protected by
*  trade secret or copyright law. Dissemination of this information or reproduction of
*  this material is strictly forbidden unless prior written permission is obtained from
*  Klepsydra Technologies GmbH.
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

kpsr::socket_mdlw::ServerSocket::ServerSocket(const std::string & socketPath)
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

