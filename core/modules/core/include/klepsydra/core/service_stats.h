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
    ServiceStats(std::string serviceName)
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
