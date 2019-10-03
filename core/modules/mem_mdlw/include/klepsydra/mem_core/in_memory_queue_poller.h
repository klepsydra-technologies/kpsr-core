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

#ifndef IN_MEMORY_QUEUE_POLLER_H
#define IN_MEMORY_QUEUE_POLLER_H

#include <map>
#include <thread>
#include <atomic>
#include <string>

#include <klepsydra/core/event_emitter.h>

#include <klepsydra/mem_core/basic_event_data.h>

namespace kpsr
{
namespace mem
{
/**
 * @brief The InMemoryQueuePoller class
 *
 * @copyright Klepsydra Technologies 2019-2020.
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
    InMemoryQueuePoller(EventEmitter & eventEmitter,
                        std::string eventName,
                        unsigned int sleepPeriodUS)
        : _running(false)
        , _eventEmitter(eventEmitter)
        , _eventName(eventName)
        , _threadNotifier()
        , _sleepPeriodUS(sleepPeriodUS)
    {}

    /**
     * @brief start
     */
    void start();

    /**
     * @brief stop
     */
    void stop();

    ~InMemoryQueuePoller();

    /**
     * @brief _running
     */
    std::atomic<bool> _running;

private:

    void pollingLoop();

    virtual void takeEventFromQueue() = 0;

protected:
    EventEmitter & _eventEmitter;
    std::string _eventName;

    std::thread _threadNotifier;
    unsigned int _sleepPeriodUS;
};
}
}
#endif
