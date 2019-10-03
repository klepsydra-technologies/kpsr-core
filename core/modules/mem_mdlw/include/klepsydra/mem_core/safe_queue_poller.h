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

#ifndef SAFE_QUEUE_POLLER_H
#define SAFE_QUEUE_POLLER_H

#include <map>
#include <thread>
#include <atomic>
#include <string>

#include <klepsydra/core/event_emitter.h>

#include <klepsydra/mem_core/safe_queue.h>
#include <klepsydra/mem_core/basic_event_data.h>
#include <klepsydra/mem_core/in_memory_queue_poller.h>

namespace kpsr
{
namespace mem
{
template <class T>
/**
 * @brief The SafeQueuePoller class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-mem-composition
 *
 * @details This class, which extends from the event_emitter_subscriber.h is an asynchronous in-memory middleware.
 * It block the subcriber thread when no events are available. This class uses the locking SafeQueue for the loop.
 *
*/
class SafeQueuePoller : public InMemoryQueuePoller
{
public:
    /**
     * @brief SafeQueuePoller
     * @param safeQueue
     * @param eventEmitter
     * @param eventName
     * @param sleepPeriodUS
     */
    SafeQueuePoller(SafeQueue <EventData<const T>> & safeQueue,
                    EventEmitter & eventEmitter,
                    std::string eventName,
                    unsigned int sleepPeriodUS)
        : InMemoryQueuePoller(eventEmitter, eventName, sleepPeriodUS)
        , _internalQueue(safeQueue)
    {}

private:

    void takeEventFromQueue() override {
        EventData<const T> event;
        bool ok = _internalQueue.timeout_move_pop(event, _sleepPeriodUS);
        if (ok) {
            _eventEmitter.emitEvent(_eventName, event.enqueuedTimeInNs, * event.eventData.get());
        }
    }
    
    SafeQueue <EventData<const T>> &_internalQueue;
};
}
}
#endif
