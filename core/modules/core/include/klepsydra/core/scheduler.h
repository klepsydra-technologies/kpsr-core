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

#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <functional>
#include <memory>
#include <string>

#include <klepsydra/core/service.h>

namespace kpsr {
class Scheduler {
public:
    virtual ~Scheduler() {}
    /**
     * @brief addToSchedule
     * @param name
     * @param after
     * @param repeat
     * @param function
     */
    virtual void startScheduledTask(const std::string & name, int after, bool repeat, std::shared_ptr<std::function<void ()>> task) = 0;

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
    virtual void stopScheduledTask(const std::string & name) = 0;

    /**
     * @brief stopScheduledService
     * @param service
     */
    virtual void stopScheduledService(Service * service) = 0;
};
}

#endif // SCHEDULER_H
