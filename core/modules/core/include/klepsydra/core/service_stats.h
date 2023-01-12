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

#ifndef SERVICE_STATS_H
#define SERVICE_STATS_H

#include <klepsydra/core/function_stats.h>

namespace kpsr {
/*!
 * @brief ServiceStats class.
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-monitoring
 *
 * @details Statistics associated to the performance of the services. The messures include the FunctionStats messures plus the total running time. A service can be dynamically stopeed and started during the execution life time of the process.
 */
struct ServiceStats : public FunctionStats
{
public:
    /*!
     * @brief ServiceStats
     * @param serviceName service name to gather stats for.
     */
    explicit ServiceStats(const std::string &serviceName)
        : FunctionStats(serviceName)
        , _totalRunningTimeMs(0)
        , _running(false)
    {}

    /*!
     * @brief start
     */
    void start() override
    {
        if (!_processingStarted) {
            _totalRunningTimeMs = 0;
            _processingStarted = true;
            this->_processingStartedTimeMs = TimeUtils::getCurrentMillisecondsAsLlu();
        }
    }

    /*!
     * @brief startTimeWatch
     */
    void startTimeWatch()
    {
        if (!_running) {
            _timetWatchMs = TimeUtils::getCurrentMillisecondsAsLlu();
        }
        _running = true;
    }

    /*!
     * @brief stopTimeWatch
     */
    void stopTimeWatch()
    {
        if (_running) {
            _totalRunningTimeMs += TimeUtils::getCurrentMillisecondsAsLlu() - _timetWatchMs;
        }
        _running = false;
    }

    /*!
     * @brief getTotalRunningTimeMs
     * @return
     */
    long long unsigned int getTotalRunningTimeMs()
    {
        if (_running) {
            return _totalRunningTimeMs + TimeUtils::getCurrentMillisecondsAsLlu() - _timetWatchMs;
        } else {
            return _totalRunningTimeMs;
        }
    }

private:
    std::atomic_ullong _totalRunningTimeMs;

    std::atomic_ullong _timetWatchMs;

    std::atomic_bool _running;
};
} // namespace kpsr

#endif // SERVICE_STATS_H
