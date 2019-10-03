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
