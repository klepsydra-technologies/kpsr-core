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

#ifndef DATA_SOCKET_H
#define DATA_SOCKET_H

#include <klepsydra/socket_core/base_socket.h>

namespace kpsr
{
namespace socket_mdlw
{
/**
 * @brief The DataSocket class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-sockets-internal
 *
 * @details A class that can read/write to a socket
 */
class DataSocket: public BaseSocket
{
public:
    /**
     * @brief DataSocket
     * @param socketId
     */
    explicit DataSocket(int socketId)
        : BaseSocket(socketId)
    {}

    template<typename F>
    /**
     * @brief getMessageData
     * @param buffer
     * @param size
     * @param scanForEnd
     */
    std::size_t getMessageData(char* buffer, std::size_t size, F scanForEnd = [](std::size_t){return false;});

    /**
     * @brief putMessageData
     * @param buffer
     * @param size
     */
    void        putMessageData(char const* buffer, std::size_t size);

    /**
     * @brief putMessageClose
     */
    void        putMessageClose();
};


}
}

#include <klepsydra/socket_core/data_socket.tpp>

#endif

