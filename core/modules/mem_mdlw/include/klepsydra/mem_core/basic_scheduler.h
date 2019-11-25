/****************************************************************************
*
*                           Klepsydra Core Modules
*              Copyright (C) 2019-2020  Klepsydra Technologies GmbH
*                            All Rights Reserved.
*
*  This file is subject to the terms and conditions defined in
*  file ‘LICENSE.md’, which is part of this source code package.
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
class BasicScheduler : public Scheduler {
public:
    void startScheduledTask(std::string name, int after, bool repeat, std::shared_ptr<std::function<void ()>> task) override;
    void startScheduledService(int after, bool repeat, Service * service) override;
    void stopScheduledTask(std::string name) override;
    void stopScheduledService(Service * service) override;

private:
    class ScheduledThread {
    public:
        ScheduledThread(int after, bool repeat, std::shared_ptr<std::function<void ()>> task);

        void start();

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
