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
    explicit EventLoopScheduler(Publisher<std::function<void()>> * publisher);

    void startScheduledTask(const std::string & name, int after, bool repeat, std::shared_ptr<std::function<void()>> function) override;
    void startScheduledService(int after, bool repeat, Service * service) override;
    void stopScheduledTask(const std::string & name) override;
    void stopScheduledService(Service * service) override;

private:
    kpsr::mem::BasicScheduler _decorableScheduler;
    Publisher<std::function<void()>> * _publisher;
};
}
}

#endif
