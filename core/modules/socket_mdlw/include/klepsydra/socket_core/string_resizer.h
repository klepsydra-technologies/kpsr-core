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

#ifndef STRING_RESIZER
#define STRING_RESIZER

#include <string>

namespace kpsr {
namespace socket_mdlw {
/**
 * @brief The StringSizer class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-sockets-internal
 *
 * @details Utility class used by getMessage() to open the string upto capacity size.
 * Then on destruction resize to the actual size of the string.
 *
 */
class StringSizer
{
    std::string&    stringData;
    std::size_t&    currentSize;
public:
    /**
     * @brief StringSizer
     * @param stringData
     * @param currentSize
     */
    StringSizer(std::string& stringData, std::size_t& currentSize)
        : stringData(stringData)
        , currentSize(currentSize) {
        stringData.resize(stringData.capacity());
    }

    ~StringSizer() {
        stringData.resize(currentSize);
    }

    /**
     * @brief incrementSize
     * @param amount
     */
    void incrementSize(std::size_t amount) {
        currentSize += amount;
    }
};
}
}

#endif // STRING_RESIZER

