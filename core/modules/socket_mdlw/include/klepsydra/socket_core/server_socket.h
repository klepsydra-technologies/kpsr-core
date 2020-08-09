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

#ifndef SERVER_SOCKET_H
#define SERVER_SOCKET_H

#include <memory>

#include <klepsydra/socket_core/base_socket.h>
#include <klepsydra/socket_core/data_socket.h>

namespace kpsr
{
namespace socket_mdlw
{
/**
 * @brief The ServerSocket class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-sockets-internal
 *
 * @details A server socket that listens on a port for a connection
 */
class ServerSocket: public BaseSocket
{
    static constexpr int maxConnectionBacklog = 5;
public:
    /**
     * @brief ServerSocket
     * @param port
     */
    explicit ServerSocket(int port);

    /**
     * @brief ServerSocket
     * @param socketPath
     */
    explicit ServerSocket(const std::string & socketPath);

    /**
     * @brief accept
     * @param timeout_us
     * @return
     *
     * @details An accepts waits for a connection and returns a
     * socket object that can be used by the client for communication
     */
    std::shared_ptr<DataSocket> accept(int timeout_us=1000000);
};

}
}

#endif

