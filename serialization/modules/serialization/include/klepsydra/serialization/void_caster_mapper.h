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

#ifndef IDENTITY_MAPPER_H
#define IDENTITY_MAPPER_H

#include <iostream>
#include <cstring>
#include <vector>

#include <klepsydra/serialization/mapper.h>

namespace kpsr
{
template <class T>
/**
 * @brief The Mapper<T, std::vector<unsigned char> > class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-serialization
 *
 */
class Mapper<T, std::vector<unsigned char>>
{
public:
    /**
     * @brief fromMiddleware
     * @param message
     * @param event
     */
    void fromMiddleware(const std::vector<unsigned char>& message, T& event) {
        std::memcpy(&event, message.data(), sizeof(T));
    }

    /**
     * @brief toMiddleware
     * @param event
     * @param message
     */
    void toMiddleware(const T& event, std::vector<unsigned char>& message) {
        std::vector<unsigned char> data((unsigned char*)(&event), (unsigned char*)&event + sizeof(T));
        message = data;
    }
};
}

#endif // IDENTITY_MAPPER_H
