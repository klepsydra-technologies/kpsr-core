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

#ifndef CONNECT_SOCKET_H
#define CONNECT_SOCKET_H

#include <klepsydra/socket_core/data_socket.h>

namespace kpsr
{
namespace socket_mdlw
{

/**
 * @brief The ConnectSocket class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-sockets-internal
 *
 * @details A class the conects to a remote machine. Allows read/write accesses to the remote machine.
 */
class ConnectSocket: public DataSocket
{
public:
    /**
     * @brief ConnectSocket
     * @param host
     * @param port
     */
    ConnectSocket(const std::string & host, int port);

    /**
     * @brief ConnectSocket
     * @param path
     */
    explicit ConnectSocket(const std::string & path);
};

}
}

#endif

