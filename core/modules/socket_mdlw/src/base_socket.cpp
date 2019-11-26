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

