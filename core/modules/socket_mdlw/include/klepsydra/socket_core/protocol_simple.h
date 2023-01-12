/*
 * Copyright 2023 Klepsydra Technologies AG
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef PROTOCOL_SIMPLE_H
#define PROTOCOL_SIMPLE_H

#include <klepsydra/socket_core/protocol.h>

namespace kpsr {
namespace socket_mdlw {
/**
 * @brief The ProtocolSimple class
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-sockets-internal
 *
 */
class ProtocolSimple : public Protocol
{
public:
    using Protocol::Protocol;
    /**
     * @brief sendMessage
     * @param message
     */
    void sendMessage(const std::string &message) override;

    /**
     * @brief recvMessage
     * @param message
     */
    void recvMessage(std::string &message) override;
};

} // namespace socket_mdlw
} // namespace kpsr

#endif
