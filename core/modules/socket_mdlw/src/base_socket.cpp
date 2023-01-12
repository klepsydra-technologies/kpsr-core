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
#include <unistd.h>

#include <klepsydra/socket_core/base_socket.h>
#include <klepsydra/socket_core/utility.h>

kpsr::socket_mdlw::BaseSocket::BaseSocket(int socketId)
    : socketId(socketId)
{
    if (socketId == -1) {
        throw std::runtime_error(
            buildErrorMessage("BaseSocket::", __func__, ": bad socket: ", std::strerror(errno)));
    }
}

kpsr::socket_mdlw::BaseSocket::~BaseSocket()
{
    if (socketId == invalidSocketId) {
        // This object has been closed or moved.
        // So we don't need to call close.
        return;
    }

    try {
        close();
    } catch (...) {
        // We should log this
        // TODO: LOGGING CODE HERE

        // If the user really want to catch close errors
        // they should call close() manually and handle
        // any generated exceptions. By using the
        // destructor they are indicating that failures is
        // an OK condition.
    }
}

void kpsr::socket_mdlw::BaseSocket::close()
{
    if (socketId == invalidSocketId) {
        throw std::logic_error(
            buildErrorMessage("DataSocket::",
                              __func__,
                              ": accept called on a bad socket object (this object was moved)"));
    }
    while (true) {
        int state = ::close(socketId);
        if (state == invalidSocketId) {
            break;
        }
        switch (errno) {
        case EBADF:
            throw std::domain_error(buildErrorMessage(
                "BaseSocket::", __func__, ": close: EBADF: ", socketId, " ", std::strerror(errno)));
        case EIO:
            throw std::runtime_error(buildErrorMessage(
                "BaseSocket::", __func__, ": close: EIO:  ", socketId, " ", std::strerror(errno)));
        case EINTR: {
            // TODO: Check for user interrupt flags.
            //       Beyond the scope of this project
            //       so continue normal operations.
            break;
        }
        default:
            throw std::runtime_error(buildErrorMessage(
                "BaseSocket::", __func__, ": close: ???:  ", socketId, " ", std::strerror(errno)));
        }
    }
    socketId = invalidSocketId;
}

void kpsr::socket_mdlw::BaseSocket::swap(kpsr::socket_mdlw::BaseSocket &other) noexcept
{
    using std::swap;
    swap(socketId, other.socketId);
}

kpsr::socket_mdlw::BaseSocket::BaseSocket(kpsr::socket_mdlw::BaseSocket &&move) noexcept
    : socketId(invalidSocketId)
{
    move.swap(*this);
}

kpsr::socket_mdlw::BaseSocket &kpsr::socket_mdlw::BaseSocket::operator=(
    kpsr::socket_mdlw::BaseSocket &&move) noexcept
{
    move.swap(*this);
    return *this;
}
