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
