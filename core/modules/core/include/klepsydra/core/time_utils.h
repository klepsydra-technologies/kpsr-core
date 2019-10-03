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

#ifndef TIME_UTILS_H
#define TIME_UTILS_H

#include <thread>

namespace kpsr {
/*!
 * @brief The TimeUtils class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-monitoring
 *
 */
class TimeUtils {
public:
    /*!
     * @brief getCurrentMilliseconds
     * @return
     */
    static long getCurrentMilliseconds() {
        std::chrono::milliseconds ms = std::chrono::duration_cast< std::chrono::milliseconds >(std::chrono::system_clock::now().time_since_epoch());
        return ms.count();
    }

    /*!
     * @brief getCurrentMillisecondsAsLlu
     * @return
     */
    static long long unsigned int getCurrentMillisecondsAsLlu() {
        std::chrono::milliseconds ms = std::chrono::duration_cast< std::chrono::milliseconds >(std::chrono::system_clock::now().time_since_epoch());
        return static_cast<long long unsigned int>(ms.count());
    }

    /*!
     * @brief getCurrentNanoseconds
     */
    static long getCurrentNanoseconds() {
        std::chrono::nanoseconds ns = std::chrono::duration_cast< std::chrono::nanoseconds >(std::chrono::system_clock::now().time_since_epoch());
        return ns.count();
    }

    /*!
     * @brief getCurrentNanosecondsAsLlu
     * @return
     */
    static long long unsigned int getCurrentNanosecondsAsLlu() {
        std::chrono::nanoseconds ns = std::chrono::duration_cast< std::chrono::nanoseconds >(std::chrono::system_clock::now().time_since_epoch());
        return static_cast<long long unsigned int>(ns.count());
    }

};
}

#endif // TIME_UTILS_H
