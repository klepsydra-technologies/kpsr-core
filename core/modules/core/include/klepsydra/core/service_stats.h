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

#ifndef SERVICE_STATS_H
#define SERVICE_STATS_H

#include <klepsydra/core/function_stats.h>

namespace kpsr {
/*!
 * @brief ServiceStats class.
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-monitoring
 *
 * @details Statistics associated to the performance of the services. The messures include the FunctionStats messures plus the total running time. A service can be dynamically stopeed and started during the execution life time of the process.
 */
struct ServiceStats : public FunctionStats {
public:

    /*!
     * @brief ServiceStats
     * @param serviceName service name to gather stats for.
     */
    explicit ServiceStats(const std::string & serviceName)
        : FunctionStats(serviceName)
        , _totalRunningTimeMs(0)
        , _running(false)
        , _started(false)
    {}

    /*!
     * @brief startTimeWatch
     */
    void startTimeWatch() {
        if (!_started) {
            _totalRunningTimeMs = 0;
            _started = true;
            this->_processingStartedTimeMs = TimeUtils::getCurrentMillisecondsAsLlu();
        }
        if (!_running) {
            _timetWatchMs = TimeUtils::getCurrentMillisecondsAsLlu();
        }
        _running = true;
    }

    /*!
     * @brief stopTimeWatch
     */
    void stopTimeWatch() {
        if (_running) {
            _totalRunningTimeMs += TimeUtils::getCurrentMillisecondsAsLlu() - _timetWatchMs;
        }
        _running = false;
    }

    /*!
     * @brief getTotalRunningTimeMs
     * @return
     */
    long long unsigned int getTotalRunningTimeMs() {
        if (_running) {
            return _totalRunningTimeMs + TimeUtils::getCurrentMillisecondsAsLlu() - _timetWatchMs;
        }
        else {
            return _totalRunningTimeMs;
        }
    }

private:

    std::atomic_ullong _totalRunningTimeMs;

    std::atomic_ullong _timetWatchMs;

    std::atomic_bool _running;

    std::atomic_bool _started;

};
}

#endif // SERVICE_STATS_H
