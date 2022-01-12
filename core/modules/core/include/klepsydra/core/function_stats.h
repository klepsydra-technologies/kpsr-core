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
 * @copyright Klepsydra Technologies 2019-2020.
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
