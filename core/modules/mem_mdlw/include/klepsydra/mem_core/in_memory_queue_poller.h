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

#ifndef IN_MEMORY_QUEUE_POLLER_H
#define IN_MEMORY_QUEUE_POLLER_H

#include <atomic>
#include <functional>
#include <future>
#include <map>
#include <thread>

#include <klepsydra/mem_core/basic_event_data.h>

namespace kpsr {
namespace mem {

static const long MEM_START_TIMEOUT_MILLISEC = 100;

/**
 * @brief The InMemoryQueuePoller class
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-mem-composition
 *
 * @details This class, provides the API to a class which extends from
 * the event_emitter_subscriber.h is an asynchronous in-memory
 * middleware.  It block the subcriber thread when no events are
 * availble. The polling loop depends on the type of queue being used
 * (locking or non-locking).
 *
*/
class InMemoryQueuePoller
{
public:
    /**
     * @brief InMemoryQueuePoller
     * @param eventEmitter
     * @param eventName
     * @param sleepPeriodUS The time in microseconds to sleep/wait
     */
    InMemoryQueuePoller(const std::string &eventName,
                        unsigned int sleepPeriodUS,
                        long timeoutMS = MEM_START_TIMEOUT_MILLISEC)
        : _running(false)
        , _started(false)
        , _eventName(eventName)
        , _threadNotifier()
        , _sleepPeriodUS(sleepPeriodUS)
        , _loopFunction(std::bind(&InMemoryQueuePoller::pollingLoop, this))
        , _threadNotifierFuture(_loopFunction.get_future())
        , _timeoutUs(timeoutMS * 1000)
    {}

    /**
     * @brief start
     */
    void start();

    /**
     * @brief stop
     */
    void stop();

    virtual ~InMemoryQueuePoller();

    /**
     * @brief _running
     */
    std::atomic<bool> _running;

    /**
     * @brief isRunning
     */
    bool isRunning();

private:
    std::atomic<bool> _started;
    bool isStarted();
    void pollingLoop();
    virtual void takeEventFromQueue() = 0;

protected:
    std::string _eventName;
    std::thread _threadNotifier;
    unsigned int _sleepPeriodUS;

private:
    std::packaged_task<void()> _loopFunction;
    std::future<void> _threadNotifierFuture;
    long _timeoutUs;
};
} // namespace mem
} // namespace kpsr
#endif
