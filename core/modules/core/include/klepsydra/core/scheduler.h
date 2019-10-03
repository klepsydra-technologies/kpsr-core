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

#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <functional>
#include <memory>
#include <string>

#include <klepsydra/core/service.h>

namespace kpsr {
class Scheduler {
public:

    /**
     * @brief addToSchedule
     * @param name
     * @param after
     * @param repeat
     * @param function
     */
    virtual void startScheduledTask(std::string name, int after, bool repeat, std::shared_ptr<std::function<void ()>> task) = 0;

    /**
     * @brief startScheduledService
     * @param after
     * @param repeat
     * @param service
     */
    virtual void startScheduledService(int after, bool repeat, Service * service) = 0;

    /**
     * @brief deleteFromSchedule
     * @param name
     */
    virtual void stopScheduledTask(std::string name) = 0;

    /**
     * @brief stopScheduledService
     * @param service
     */
    virtual void stopScheduledService(Service * service) = 0;
};
}

#endif // SCHEDULER_H
