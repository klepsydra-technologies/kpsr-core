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

#include <klepsydra/socket_core/data_socket.h>
#include <klepsydra/socket_core/utility.h>

void kpsr::socket_mdlw::DataSocket::putMessageData(char const *buffer, std::size_t size)
{
    std::size_t dataWritten = 0;

    while (dataWritten < size) {
        std::size_t put = write(getSocketId(), buffer + dataWritten, size - dataWritten);
        if (put == static_cast<std::size_t>(-1)) {
            switch (errno) {
            case EINVAL:
            case EBADF:
            case ECONNRESET:
            case ENXIO:
            case EPIPE: {
                // Fatal error. Programming bug
                throw std::domain_error(buildErrorMessage("DataSocket::",
                                                          __func__,
                                                          ": write: critical error: ",
                                                          strerror(errno)));
            }
            case EDQUOT:
            case EFBIG:
            case EIO:
            case ENETDOWN:
            case ENETUNREACH:
            case ENOSPC: {
                // Resource acquisition failure or device error
                throw std::runtime_error(buildErrorMessage("DataSocket::",
                                                           __func__,
                                                           ": write: resource failure: ",
                                                           strerror(errno)));
            }
            case EINTR:
                // TODO: Check for user interrupt flags.
                //       Beyond the scope of this project
                //       so continue normal operations.
            case EAGAIN: {
                // Temporary error.
                // Simply retry the read.
                continue;
            }
            default: {
                throw std::runtime_error(buildErrorMessage("DataSocket::",
                                                           __func__,
                                                           ": write: returned -1: ",
                                                           strerror(errno)));
            }
            }
        }
        dataWritten += put;
    }
    return;
}

void kpsr::socket_mdlw::DataSocket::putMessageClose()
{
    if (::shutdown(getSocketId(), SHUT_WR) != 0) {
        throw std::domain_error(buildErrorMessage("HTTPProtocol::",
                                                  __func__,
                                                  ": shutdown: critical error: ",
                                                  strerror(errno)));
    }
}
