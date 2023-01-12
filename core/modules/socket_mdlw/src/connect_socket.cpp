// Copyright 2023 Klepsydra Technologies AG
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <arpa/inet.h>
#include <cstring>
#include <sys/un.h>
#include <unistd.h>

#include <iostream>

#include <klepsydra/socket_core/connect_socket.h>
#include <klepsydra/socket_core/utility.h>

kpsr::socket_mdlw::ConnectSocket::ConnectSocket(const std::string &host, int port)
    : kpsr::socket_mdlw::DataSocket(::socket(PF_INET, SOCK_STREAM, 0))
{
    struct sockaddr_in serverAddr
    {};
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(port);
    serverAddr.sin_addr.s_addr = inet_addr(host.c_str());

    if (::connect(getSocketId(), (struct sockaddr *) &serverAddr, sizeof(serverAddr)) != 0) {
        close();
        throw std::runtime_error(
            buildErrorMessage("ConnectSocket::", __func__, ": connect: ", strerror(errno)));
    }
}

kpsr::socket_mdlw::ConnectSocket::ConnectSocket(const std::string &path)
    : kpsr::socket_mdlw::DataSocket(::socket(AF_UNIX, SOCK_STREAM, 0))
{
    struct sockaddr_un serverAddr
    {};
    serverAddr.sun_family = AF_UNIX;
    strncpy(serverAddr.sun_path, path.data(), sizeof(serverAddr.sun_path) - 1);

    if (::connect(getSocketId(), (struct sockaddr *) &serverAddr, sizeof(serverAddr)) != 0) {
        close();
        throw std::runtime_error(
            buildErrorMessage("ConnectSocket::", __func__, ": connect: ", strerror(errno)));
    }
}
