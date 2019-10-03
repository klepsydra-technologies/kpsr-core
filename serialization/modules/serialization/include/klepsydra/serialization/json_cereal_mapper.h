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

#ifndef CEREAL_JSON_MAPPER_H
#define CEREAL_JSON_MAPPER_H

#include <sstream>

#include <cereal/cereal.hpp>
#include <cereal/archives/json.hpp>

#include <klepsydra/serialization/mapper.h>

namespace kpsr
{
template <class T>
/**
 * @brief The Mapper<T, std::string> class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-serialization-cereal
 *
 */
class Mapper<T, std::string>
{
public:
    /**
     * @brief fromMiddleware
     * @param message
     * @param event
     */
    void fromMiddleware(const std::string& message, T& event) {
        std::stringstream ss;
        ss.str(message);
        cereal::JSONInputArchive archive(ss);
        archive(CEREAL_NVP(event));
    }

    /**
     * @brief toMiddleware
     * @param event
     * @param message
     */
    void toMiddleware(const T& event, std::string& message) {
        std::stringstream ss;
        {
            cereal::JSONOutputArchive archive( ss );
            archive(CEREAL_NVP(event));
        }
        message = ss.str();
    }
};
}

#endif // CEREAL_JSON_MAPPER_H
