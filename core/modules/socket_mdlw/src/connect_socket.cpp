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
#include <cstring>

#include <iostream>

#include <klepsydra/socket_core/connect_socket.h>
#include <klepsydra/socket_core/utility.h>

kpsr::socket_mdlw::ConnectSocket::ConnectSocket(const std::string & host, int port)
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

kpsr::socket_mdlw::ConnectSocket::ConnectSocket(const std::string & path)
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
