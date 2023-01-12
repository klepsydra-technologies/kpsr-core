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

#ifndef BASIC_SCHEDULER_H
#define BASIC_SCHEDULER_H

#include <map>
#include <memory>
#include <thread>

#include <klepsydra/core/scheduler.h>

namespace kpsr {
namespace mem {
/**
 * @brief The Basic Scheduler class
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version   2.1.0
 *
 */
class BasicScheduler : public Scheduler
{
public:
    /**
     * @brief Start Scheduled Task
     * @param name Name of task
     * @param after Start after uS
     * @param repeat Repeat task
     * @param task Task.
     */
    void startScheduledTask(const std::string &name,
                            int after,
                            bool repeat,
                            std::function<void()> task) override;
    /**
     * @brief Start Service
     * @param after Start after uS
     * @param repeat Repeat service function
     * @param service Service to execute
     */

    void startScheduledService(int after, bool repeat, Service *service) override;
    /**
     * @brief Stop Task
     * @param name Name of task
     */
    void stopScheduledTask(const std::string &name) override;
    /**
     * @brief Stop Service
     * @param service Service to stop
     */
    void stopScheduledService(Service *service) override;

private:
    /**
     * @brief Internel Thread class
     */
    class ScheduledThread
    {
    public:
        /**
         * @brief ScheduledThread
         *
         * @param after Start after this many uS
         * @param repeat Repeat task
         * @param task Task
         */
        ScheduledThread(int after, bool repeat, std::function<void()> task);

        /**
         * @brief
         */
        void start();

        /**
         * @brief
         */
        void stop();

    private:
        int _after;
        bool _repeat;
        bool _isRunning;
        std::thread _thread;
        std::function<void()> _task;
    };

    std::map<std::string, std::shared_ptr<ScheduledThread>> _threadMap;
};
} // namespace mem
} // namespace kpsr

#endif // BASIC_SCHEDULER_H
