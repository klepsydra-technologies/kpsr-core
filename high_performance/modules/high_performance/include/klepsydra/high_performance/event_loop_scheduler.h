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

#ifndef EVENT_LOOP_SCHEDULER_H
#define EVENT_LOOP_SCHEDULER_H

#include <klepsydra/core/publisher.h>
#include <klepsydra/mem_core/basic_scheduler.h>

#include <functional>
#include <chrono>
#include <thread>
#include <atomic>
#include <memory>

namespace kpsr {
namespace high_performance {
/**
 * @brief The Scheduler class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version 2.0.1
 *
 * @ingroup kpsr-eventloop-internal
 *
 */
class EventLoopScheduler : public Scheduler
{
public:
    EventLoopScheduler(Publisher<std::function<void()>> * publisher);

    void startScheduledTask(std::string name, int after, bool repeat, std::shared_ptr<std::function<void()>> function) override;
    void startScheduledService(int after, bool repeat, Service * service) override;
    void stopScheduledTask(std::string name) override;
    void stopScheduledService(Service * service) override;

private:
    kpsr::mem::BasicScheduler _decorableScheduler;
    Publisher<std::function<void()>> * _publisher;
};
}
}

#endif
