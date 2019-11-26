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

