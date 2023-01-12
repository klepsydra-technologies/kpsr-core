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

#ifndef KPSR_FUNC_STATS_H
#define KPSR_FUNC_STATS_H

#include <atomic>
#include <string>

#include <klepsydra/core/time_utils.h>

#include <spdlog/spdlog.h>

namespace kpsr {
/*!
 * @brief FunctionStats class.
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-monitoring
 *
 * @details Statistics associated to the performance of a customer function. It gathers three main messures: number of invocations, total invocation time, starting time.
 * The use of this class is very simple can be split into three parts: construct, attach to container and surround the code block to be messured. This can be seen in the following example:
@code
class MeasuredClass {
public:
   MeasuredClass(Container * container)
      : functionStats("measured_class")
   {
      if (container) {
         container->attach(&functionStats);
      }
   }

   void measuredFunction() {
      functionStats.startProcessMeasure();
      // DO STUFF
      functionStats.stopProcessMeasure();
   }
private:
   FunctionStats functionStats;
}
@endcode
 */
struct FunctionStats
{
    explicit FunctionStats(const std::string &name)
        : name(name)
        , _beforeTimeNs(0)
        , _processingStarted(false)
        , _processingStartedTimeMs(0)
    {}

    /*!
     * @brief name
     */
    const std::string name;

    /*!
     * @brief totalProcessed
     */
    long long unsigned int totalProcessed = 0;

    /*!
     * @brief totalProcessingTimeInNanoSecs
     */
    long long unsigned int totalProcessingTimeInNanoSecs = 0;

    /*!
     * @brief getMillisecondsSinceCreation
     * @return
     */
    long long unsigned int getMillisecondsSinceCreation()
    {
        return TimeUtils::getCurrentMillisecondsAsLlu() - _creationTimeMs;
    }

    /*!
     * \brief start
     */
    virtual void start()
    {
        if (!_processingStarted) {
            spdlog::debug("{}. {}", __PRETTY_FUNCTION__, name);
            _processingStartedTimeMs = TimeUtils::getCurrentMillisecondsAsLlu();
            _processingStarted = true;
        }
    }

    /*!
     * \brief stop
     */
    virtual void stop()
    {
        if (_processingStarted) {
            spdlog::debug("{}. {}", __PRETTY_FUNCTION__, name);
            _processingStarted = false;
        }
    }

    /*!
     * @brief getMillisecondsSinceStart
     * @return
     */
    long long unsigned int getMillisecondsSinceStart()
    {
        if (_processingStartedTimeMs != 0) {
            return TimeUtils::getCurrentMillisecondsAsLlu() - _processingStartedTimeMs;
        }
        return 0;
    }

    void startProcessMeasure() { _beforeTimeNs = TimeUtils::getCurrentNanosecondsAsLlu(); }

    void stopProcessMeasure()
    {
        totalProcessingTimeInNanoSecs += TimeUtils::getCurrentNanosecondsAsLlu() - _beforeTimeNs;
        totalProcessed++;
    }

protected:
    const long long unsigned int _creationTimeMs = TimeUtils::getCurrentMillisecondsAsLlu();

    std::atomic_ullong _beforeTimeNs;

    std::atomic_bool _processingStarted;

    std::atomic_ullong _processingStartedTimeMs;
};

} // namespace kpsr

#endif // KPSR_FUNC_STATS_H
