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

#ifndef BASIC_SCHEDULER_H
#define BASIC_SCHEDULER_H

#include <thread>
#include <map>
#include <memory>

#include <klepsydra/core/scheduler.h>

namespace kpsr {
namespace mem {
/**
 * @brief The Basic Scheduler class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 */
class BasicScheduler : public Scheduler {
public:
    /**
     * @brief Start Scheduled Task
     * @param name Name of task
     * @param after Start after uS
     * @param repeat Repeat task
     * @param task Task.
     */
    void startScheduledTask(const std::string & name, int after, bool repeat, std::shared_ptr<std::function<void ()>> task) override;
    /**
     * @brief Start Service
     * @param after Start after uS
     * @param repeat Repeat service function
     * @param service Service to execute
     */
    void startScheduledService(int after, bool repeat, Service * service) override;
    /**
     * @brief Stop Task
     * @param name Name of task
     */
    void stopScheduledTask(const std::string & name) override;
    /**
     * @brief Stop Service
     * @param service Service to stop
     */
    void stopScheduledService(Service * service) override;

private:
    /**
     * @brief Internel Thread class
     */
    class ScheduledThread {
    public:
        /**
         * @brief ScheduledThread
         *
         * @param after Start after this many uS
         * @param repeat Repeat task
         * @param task Task
         */
        ScheduledThread(int after, bool repeat, std::shared_ptr<std::function<void ()>> task);

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
        std::shared_ptr<std::function<void ()>> _task;
    };

    std::map<std::string, std::shared_ptr<ScheduledThread>> _threadMap;
};
}
}

#endif // BASIC_SCHEDULER_H
