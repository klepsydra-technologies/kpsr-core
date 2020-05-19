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

#include <klepsydra/socket_core/protocol_simple.h>
#include <klepsydra/socket_core/string_resizer.h>

void kpsr::socket_mdlw::ProtocolSimple::sendMessage(const std::string & message)
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
