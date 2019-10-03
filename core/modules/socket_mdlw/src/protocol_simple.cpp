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

#include <klepsydra/socket_core/protocol_simple.h>
#include <klepsydra/socket_core/string_resizer.h>

void kpsr::socket_mdlw::ProtocolSimple::sendMessage(std::string const& message)
{
    _dataSocket->putMessageData(message.c_str(), message.size());
    _dataSocket->putMessageClose();
}

void kpsr::socket_mdlw::ProtocolSimple::recvMessage(std::string& message)
{
    std::size_t     dataRead = 0;
    message.clear();

    while(true)
    {
        // This outer loop handles resizing of the message when we run of space in the string.
        kpsr::socket_mdlw::StringSizer stringSizer(message, dataRead);
        std::size_t const              dataMax  = message.capacity() - 1;
        char*                          buffer   = &message[0];

        std::size_t got = _dataSocket->getMessageData(buffer + dataRead, dataMax - dataRead, [](std::size_t){return false;});
        dataRead    += got;
        if (got == 0)
        {
            break;
        }

        // Resize the string buffer
        // So that next time around we can read more data.
        message.reserve(message.capacity() * 1.5 + 10);
    }
}
