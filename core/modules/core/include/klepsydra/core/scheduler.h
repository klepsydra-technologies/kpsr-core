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

#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <functional>
#include <memory>
#include <string>

#include <klepsydra/core/service.h>

namespace kpsr {
class Scheduler
{
public:
    virtual ~Scheduler() {}
    /**
     * @brief addToSchedule
     * @param name
     * @param after
     * @param repeat
     * @param function
     */
    virtual void startScheduledTask(const std::string &name,
                                    int after,
                                    bool repeat,
                                    std::function<void()> task) = 0;

    /**
     * @brief startScheduledService
     * @param after
     * @param repeat
     * @param service
     */
    virtual void startScheduledService(int after, bool repeat, Service *service) = 0;

    /**
     * @brief deleteFromSchedule
     * @param name
     */
    virtual void stopScheduledTask(const std::string &name) = 0;

    /**
     * @brief stopScheduledService
     * @param service
     */
    virtual void stopScheduledService(Service *service) = 0;
};
} // namespace kpsr

#endif // SCHEDULER_H
