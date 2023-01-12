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

#ifndef CONCURRENT_QUEUE_POLLER_H
#define CONCURRENT_QUEUE_POLLER_H

#include <atomic>
#include <map>
#include <string>
#include <thread>

#include <klepsydra/core/safe_event_emitter.h>

#include <klepsydra/mem_core/basic_event_data.h>
#include <klepsydra/mem_core/in_memory_queue_poller.h>

#include <concurrentqueue.h>

namespace kpsr {
namespace mem {
template<class T>
/**
 * @brief The ConcurrentQueuePoller class
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-mem-composition
 *
 * @details This class, which extends from the
 * event_emitter_subscriber.h is an asynchronous in-memory middleware.
 * It blocks the subcriber thread when no events are available. This
 * class uses the non-locking concurrent queue.
 *
*/
class ConcurrentQueuePoller : public InMemoryQueuePoller
{
public:
    /**
     * @brief ConcurrentQueuePoller
     * @param concurrentQueue
     * @param eventEmitter
     * @param eventName
     * @param sleepPeriodUS The time in microseconds to sleep/wait
     * @param token The producer token used by the publisher
     */
    ConcurrentQueuePoller(
        moodycamel::ConcurrentQueue<EventData<const T>> &concurrentQueue,
        std::shared_ptr<EventEmitterInterface<std::shared_ptr<const T>>> &eventEmitter,
        std::string eventName,
        unsigned int sleepPeriodUS,
        moodycamel::ProducerToken &token)
        : InMemoryQueuePoller(eventName, sleepPeriodUS)
        , _eventEmitter(eventEmitter)
        , _internalQueue(concurrentQueue)
        , _token(token)
    {}

private:
    void takeEventFromQueue() override
    {
        EventData<const T> event;
        bool ok = _internalQueue.try_dequeue_from_producer(_token, event);
        if (ok) {
            _eventEmitter->emitEvent(_eventName, event.enqueuedTimeInNs, event.eventData);
        } else {
            std::this_thread::sleep_for(std::chrono::microseconds(_sleepPeriodUS));
        }
    }

    std::shared_ptr<EventEmitterInterface<std::shared_ptr<const T>>> _eventEmitter;
    moodycamel::ConcurrentQueue<EventData<const T>> &_internalQueue;
    moodycamel::ProducerToken &_token;
};
} // namespace mem
} // namespace kpsr
#endif
