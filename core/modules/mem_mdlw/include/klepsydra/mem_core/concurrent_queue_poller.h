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

#ifndef CONCURRENT_QUEUE_POLLER_H
#define CONCURRENT_QUEUE_POLLER_H

#include <map>
#include <thread>
#include <atomic>
#include <string>

#include <klepsydra/core/event_emitter.h>

#include <klepsydra/mem_core/basic_event_data.h>
#include <klepsydra/mem_core/in_memory_queue_poller.h>

#include <concurrentqueue.h>

namespace kpsr
{
namespace mem
{
template <class T>
/**
 * @brief The ConcurrentQueuePoller class
 *
 * @copyright Klepsydra Technologies 2019-2020.
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
    ConcurrentQueuePoller(moodycamel::ConcurrentQueue <EventData<const T>> & concurrentQueue,
                          EventEmitter & eventEmitter,
                          std::string eventName,
                          unsigned int sleepPeriodUS,
                          moodycamel::ProducerToken & token)
        : InMemoryQueuePoller(eventEmitter, eventName, sleepPeriodUS)
        , _internalQueue(concurrentQueue)
        , _token(token)
    {}

private:

    void takeEventFromQueue() override {
        EventData<const T> event;
        bool ok = _internalQueue.try_dequeue_from_producer(_token, event);
        if (ok) {
            _eventEmitter.emitEvent(_eventName, event.enqueuedTimeInNs, * event.eventData.get());
        }
        else {
            std::this_thread::sleep_for(std::chrono::microseconds(_sleepPeriodUS));
        }
    }


    moodycamel::ConcurrentQueue <EventData<const T>> &_internalQueue;
    moodycamel::ProducerToken & _token;
};
}
}
#endif
