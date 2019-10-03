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
