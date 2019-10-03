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

#include <stdexcept>
#include <unistd.h>
#include <cstring>

#include <klepsydra/socket_core/utility.h>

namespace kpsr
{
namespace socket_mdlw
{

template<typename F>
/**
 * @brief DataSocket::getMessageData
 * @param buffer
 * @param size
 * @param scanForEnd
 * @return
 */
std::size_t DataSocket::getMessageData(char* buffer, std::size_t size, F scanForEnd)
{
    if (getSocketId() == 0)
    {
        throw std::logic_error(buildErrorMessage("DataSocket::", __func__, ": accept called on a bad socket object (this object was moved)"));
    }

    std::size_t     dataRead  = 0;
    while(dataRead < size)
    {
        // The inner loop handles interactions with the socket.
        std::size_t get = read(getSocketId(), buffer + dataRead, size - dataRead);
        if (get == static_cast<std::size_t>(-1))
        {
            switch(errno)
            {
            case EBADF:
            case EFAULT:
            case EINVAL:
            case ENXIO:
            {
                // Fatal error. Programming bug
                throw std::domain_error(buildErrorMessage("DataSocket::", __func__, ": read: critical error: ", std::strerror(errno)));
            }
            case EIO:
            case ENOBUFS:
            case ENOMEM:
            {
                // Resource acquisition failure or device error
                throw std::runtime_error(buildErrorMessage("DataSocket::", __func__, ": read: resource failure: ", std::strerror(errno)));
            }
            case EINTR:
                // TODO: Check for user interrupt flags.
                //       Beyond the scope of this project
                //       so continue normal operations.
            case ETIMEDOUT:
            case EAGAIN:
            {
                // Temporary error.
                // Simply retry the read.
                continue;
            }
            case ECONNRESET:
            case ENOTCONN:
            {
                // Connection broken.
                // Return the data we have available and exit
                // as if the connection was closed correctly.
                get = 0;
                break;
            }
            default:
            {
                throw std::runtime_error(buildErrorMessage("DataSocket::", __func__, ": read: returned -1: ", std::strerror(errno)));
            }
            }
        }
        if (get == 0)
        {
            break;
        }
        dataRead += get;
        if (scanForEnd(dataRead))
        {
            break;
        }
    }

    return dataRead;
}

}
}

