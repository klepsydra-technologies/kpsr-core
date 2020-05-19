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

#ifndef PROTOCOL_SIMPLE_H
#define PROTOCOL_SIMPLE_H

#include <klepsydra/socket_core/protocol.h>

namespace kpsr
{
namespace socket_mdlw
{
/**
 * @brief The ProtocolSimple class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-sockets-internal
 *
 */
class ProtocolSimple: public Protocol
{
public:
    using Protocol::Protocol;
    /**
     * @brief sendMessage
     * @param message
     */
    void sendMessage(const std::string & message) override;

    /**
     * @brief recvMessage
     * @param message
     */
    void recvMessage(std::string& message) override;
};

}
}

#endif

