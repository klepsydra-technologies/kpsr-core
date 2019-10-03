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

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <string>
#include <memory>

#include <klepsydra/socket_core/data_socket.h>

namespace kpsr
{
namespace socket_mdlw
{
/**
 * @brief The Protocol class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-sockets-internal
 *
 */
class Protocol
{
protected:
    std::shared_ptr<DataSocket> _dataSocket;
public:
    /**
     * @brief Protocol
     * @param dataSocket
     */
    Protocol(std::shared_ptr<DataSocket> dataSocket);

    ~Protocol();

    /**
     * @brief sendMessage
     * @param message
     */
    virtual void sendMessage(std::string const& message) = 0;

    /**
     * @brief recvMessage
     * @param message
     */
    virtual void recvMessage(std::string& message) = 0;
};

}
}

#endif

