// Copyright 2023 Klepsydra Technologies AG
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <klepsydra/socket_core/protocol_simple.h>
#include <klepsydra/socket_core/string_resizer.h>

void kpsr::socket_mdlw::ProtocolSimple::sendMessage(const std::string &message)
{
    _dataSocket->putMessageData(message.c_str(), message.size());
    _dataSocket->putMessageClose();
}

void kpsr::socket_mdlw::ProtocolSimple::recvMessage(std::string &message)
{
    std::size_t dataRead = 0;
    message.clear();

    while (true) {
        // This outer loop handles resizing of the message when we run of space in the string.
        kpsr::socket_mdlw::StringSizer stringSizer(message, dataRead);
        std::size_t const dataMax = message.capacity() - 1;
        char *buffer = &message[0];

        std::size_t got = _dataSocket->getMessageData(buffer + dataRead,
                                                      dataMax - dataRead,
                                                      [](std::size_t) { return false; });
        dataRead += got;
        if (got == 0) {
            break;
        }

        // Resize the string buffer
        // So that next time around we can read more data.
        message.reserve(message.capacity() * 1.5 + 10);
    }
}
