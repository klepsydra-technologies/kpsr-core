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
#include <unistd.h>
#include <cstring>

#include <klepsydra/socket_core/data_socket.h>
#include <klepsydra/socket_core/utility.h>

void kpsr::socket_mdlw::DataSocket::putMessageData(char const* buffer, std::size_t size)
{
    std::size_t     dataWritten = 0;

    while(dataWritten < size)
    {
        std::size_t put = write(getSocketId(), buffer + dataWritten, size - dataWritten);
        if (put == static_cast<std::size_t>(-1))
        {
            switch(errno)
            {
            case EINVAL:
            case EBADF:
            case ECONNRESET:
            case ENXIO:
            case EPIPE:
            {
                // Fatal error. Programming bug
                throw std::domain_error(buildErrorMessage("DataSocket::", __func__, ": write: critical error: ", strerror(errno)));
            }
            case EDQUOT:
            case EFBIG:
            case EIO:
            case ENETDOWN:
            case ENETUNREACH:
            case ENOSPC:
            {
                // Resource acquisition failure or device error
                throw std::runtime_error(buildErrorMessage("DataSocket::", __func__, ": write: resource failure: ", strerror(errno)));
            }
            case EINTR:
                // TODO: Check for user interrupt flags.
                //       Beyond the scope of this project
                //       so continue normal operations.
            case EAGAIN:
            {
                // Temporary error.
                // Simply retry the read.
                continue;
            }
            default:
            {
                throw std::runtime_error(buildErrorMessage("DataSocket::", __func__, ": write: returned -1: ", strerror(errno)));
            }
            }
        }
        dataWritten += put;
    }
    return;
}

void kpsr::socket_mdlw::DataSocket::putMessageClose()
{
    if (::shutdown(getSocketId(), SHUT_WR) != 0)
    {
        throw std::domain_error(buildErrorMessage("HTTPProtocol::", __func__, ": shutdown: critical error: ", strerror(errno)));
    }
}

