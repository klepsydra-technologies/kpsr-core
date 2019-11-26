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

