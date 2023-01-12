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

#ifndef EVENT_LOOP_SCHEDULER_H
#define EVENT_LOOP_SCHEDULER_H

#include <klepsydra/core/publisher.h>
#include <klepsydra/mem_core/basic_scheduler.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <thread>

namespace kpsr {
namespace high_performance {
/**
 * @brief The Scheduler class
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version 2.0.1
 *
 * @ingroup kpsr-eventloop-internal
 *
 */
class EventLoopScheduler : public Scheduler
{
public:
    explicit EventLoopScheduler(Publisher<std::function<void()>> *publisher);

    void startScheduledTask(const std::string &name,
                            int after,
                            bool repeat,
                            std::function<void()> function) override;
    void startScheduledService(int after, bool repeat, Service *service) override;
    void stopScheduledTask(const std::string &name) override;
    void stopScheduledService(Service *service) override;

private:
    kpsr::mem::BasicScheduler _decorableScheduler;
    Publisher<std::function<void()>> *_publisher;
};
} // namespace high_performance
} // namespace kpsr

#endif
