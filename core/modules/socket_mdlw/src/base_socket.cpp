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

#include <klepsydra/socket_core/base_socket.h>
#include <klepsydra/socket_core/utility.h>

kpsr::socket_mdlw::BaseSocket::BaseSocket(int socketId)
    : socketId(socketId)
{
    if (socketId == -1)
    {
        throw std::runtime_error(buildErrorMessage("BaseSocket::", __func__, ": bad socket: ", std::strerror(errno)));
    }
}

kpsr::socket_mdlw::BaseSocket::~BaseSocket()
{
    if (socketId == invalidSocketId)
    {
        // This object has been closed or moved.
        // So we don't need to call close.
        return;
    }

    try
    {
        close();
    }
    catch(...)
    {
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
    if (socketId == invalidSocketId)
    {
        throw std::logic_error(buildErrorMessage("DataSocket::", __func__, ": accept called on a bad socket object (this object was moved)"));
    }
    while(true)
    {
        int state = ::close(socketId);
        if (state == invalidSocketId)
        {
            break;
        }
        switch(errno)
        {
            case EBADF: throw std::domain_error(buildErrorMessage("BaseSocket::", __func__, ": close: EBADF: ", socketId, " ", std::strerror(errno)));
            case EIO:   throw std::runtime_error(buildErrorMessage("BaseSocket::", __func__, ": close: EIO:  ", socketId, " ", std::strerror(errno)));
            case EINTR:
            {
                        // TODO: Check for user interrupt flags.
                        //       Beyond the scope of this project
                        //       so continue normal operations.
                break;
            }
            default:    throw std::runtime_error(buildErrorMessage("BaseSocket::", __func__, ": close: ???:  ", socketId, " ", std::strerror(errno)));
        }
    }
    socketId = invalidSocketId;
}

void kpsr::socket_mdlw::BaseSocket::swap(kpsr::socket_mdlw::BaseSocket& other) noexcept
{
    using std::swap;
    swap(socketId,   other.socketId);
}

kpsr::socket_mdlw::BaseSocket::BaseSocket(kpsr::socket_mdlw::BaseSocket&& move) noexcept
    : socketId(invalidSocketId)
{
    move.swap(*this);
}

kpsr::socket_mdlw::BaseSocket& kpsr::socket_mdlw::BaseSocket::operator=(kpsr::socket_mdlw::BaseSocket&& move) noexcept
{
    move.swap(*this);
    return *this;
}

