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
