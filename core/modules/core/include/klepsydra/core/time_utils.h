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

#ifndef TIME_UTILS_H
#define TIME_UTILS_H

#include <thread>

namespace kpsr {
/*!
 * @brief The TimeUtils class
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-monitoring
 *
 */
class TimeUtils
{
public:
    /*!
     * @brief getCurrentMilliseconds
     * @return
     */
    static long getCurrentMilliseconds()
    {
        std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch());
        return ms.count();
    }

    /*!
     * @brief getCurrentMillisecondsAsLlu
     * @return
     */
    static long long unsigned int getCurrentMillisecondsAsLlu()
    {
        std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch());
        return static_cast<long long unsigned int>(ms.count());
    }

    /*!
     * @brief getCurrentNanoseconds
     */
    static long getCurrentNanoseconds()
    {
        std::chrono::nanoseconds ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch());
        return ns.count();
    }

    /*!
     * @brief getCurrentNanosecondsAsLlu
     * @return
     */
    static long long unsigned int getCurrentNanosecondsAsLlu()
    {
        std::chrono::nanoseconds ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch());
        return static_cast<long long unsigned int>(ns.count());
    }
};
} // namespace kpsr

#endif // TIME_UTILS_H
