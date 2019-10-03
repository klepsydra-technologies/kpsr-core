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

#ifndef KPSR_FUNC_STATS_H
#define KPSR_FUNC_STATS_H

#include <atomic>
#include <string>

#include <klepsydra/core/time_utils.h>

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
class MessuredClass {
public:
   MessuredClass(Container * container)
      : functionStats("messured_class")
   {
      if (container) {
         container->attach(&functionStats);
      }
   }

   void meesuredFunction() {
      functionStats.startProcessMeassure();
      // DO STUFF
      functionStats.stopProcessMeassure();
   }
private:
   FunctionStats functionStats;
}
@endcode
 */
struct FunctionStats
{
    FunctionStats(const std::string name)
        : _name(name)
        , _processingStarted(false)
    {}

    /*!
     * @brief _name
     */
    const std::string _name;

    /*!
     * @brief _totalProcessed
     */
    long long unsigned int _totalProcessed = 0;

    /*!
     * @brief _totalProcessingTimeInNanoSecs
     */
    long long unsigned int _totalProcessingTimeInNanoSecs = 0;

    /*!
     * @brief getMillisecondsSinceCreation
     * @return
     */
    long long unsigned int getMillisecondsSinceCreation() {
        return TimeUtils::getCurrentMillisecondsAsLlu() - _creationTimeMs;
    }

    /*!
     * @brief getMillisecondsSinceStart
     * @return
     */
    long long unsigned int getMillisecondsSinceStart() {
        if (_processingStarted) {
            return TimeUtils::getCurrentMillisecondsAsLlu() - _processingStartedTimeMs;
        }
        return 0;
    }

    void startProcessMeassure() {
        _beforeTimeNs = TimeUtils::getCurrentNanosecondsAsLlu();
        if (!_processingStarted) {
            _processingStartedTimeMs = TimeUtils::getCurrentMillisecondsAsLlu();
            _processingStarted = true;
        }
    }

    void stopProcessMeassure() {
        _totalProcessingTimeInNanoSecs += TimeUtils::getCurrentNanosecondsAsLlu() - _beforeTimeNs;
        _totalProcessed++;
    }

protected:

    const long long unsigned int _creationTimeMs = TimeUtils::getCurrentMillisecondsAsLlu();

    std::atomic_ullong _beforeTimeNs;

    std::atomic_bool _processingStarted;

    std::atomic_ullong _processingStartedTimeMs;
};

}

#endif // KPSR_FUNC_STATS_H
