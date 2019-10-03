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

#ifndef CEREAL_BINARY_MAPPER_H
#define CEREAL_BINARY_MAPPER_H

#include <memory>
#include <streambuf>

#include <cereal/cereal.hpp>
#include <cereal/archives/portable_binary.hpp>
#include <cereal/types/string.hpp>

#include <klepsydra/serialization/mapper.h>

using Base = std::basic_streambuf<char> *;

namespace kpsr
{
template <class T>
/**
 * @brief The Mapper<T, Base> class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-serialization-cereal
 *
 */
class Mapper<T, Base>
{
public:
    /**
     * @brief fromMiddleware
     * @param message
     * @param event
     */
    void fromMiddleware(const Base & message, T& event) {
        std::istream iss(message);
        cereal::PortableBinaryInputArchive archive(iss);
        archive(CEREAL_NVP(event));
    }

    /**
     * @brief toMiddleware
     * @param event
     * @param message
     */
    void toMiddleware(const T& event, Base & message) {
        std::ostream oss(message);
        {
            cereal::PortableBinaryOutputArchive archive(oss);
            archive(CEREAL_NVP(event));
        }
    }
};
}

#endif // CEREAL_BINARY_MAPPER_H
