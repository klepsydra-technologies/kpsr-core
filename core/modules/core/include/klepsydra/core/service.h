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

#ifndef SERVICE_H
#define SERVICE_H

#include <atomic>
#include <map>

#include <klepsydra/core/environment.h>
#include <klepsydra/core/service_stats.h>

namespace kpsr
{
/*!
 * @brief The Service class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-application
 *
 * @details Service Interface. Custumer applications using Klepsydra that need remote administration, access to
 * configuration perform regular task, etc should implement this interface.
 *
 * The only pure virtual methods to implement are start, stop and execute.
 * Master services start running as soon as the application is _started and should not implement the start method.
 * There are some stat gathering helper functions that are used within the container interface.
 */
class Service
{
public:
    /*!
     * @brief Service
     * @param environment
     * @param serviceName
     * @param isMaster
     */
    Service(Environment * environment,
            std::string serviceName,
            bool isMaster = true)
        : _environment(environment)
        , _serviceStats(serviceName)
        , _isMaster(isMaster)
        , _started(false)
    {}

    /*!
     * @brief runOnce
     *
     * This public method is used to invoke the custom service execute method.
     * Usually this method is invoked within a main application. E.g., in the Ros world, this will be invoked within
     * the runOnce of Ros.
     */
    virtual void runOnce() final {
        if (_started) {
            _serviceStats.startProcessMeassure();
            execute();
            _serviceStats.stopProcessMeassure();
        }
    }

    /*!
     * @brief startup
     *
     * This public method is used to invoke the custom service start method.
     * Usually this method is invoked within a main application.
     */
    virtual void startup() final {
        _started = true;

        _serviceStats.startTimeWatch();

        start();
    }

    /*!
     * @brief shutdown
     *
     * This public method is used to invoke the custom service stop method.
     * Usually this method is invoked within a main application.
     */
    virtual void shutdown() final {
        _started = false;

        _serviceStats.stopTimeWatch();

        stop();
    }

    /*!
     * @brief getMetadata
     *
     * This method is intented to be used from the container. It provides metadata related to the service including status,
     * running time, etc.
     *
     * @return
     */
    virtual std::map<std::string, std::string> getMetadata() {
        std::map<std::string, std::string> metadata;
        return metadata;
    }

    /*!
     * @brief _environment
     */
    Environment * _environment;

    /*!
     * @brief _serviceStats
     */
    ServiceStats _serviceStats;

    /*!
     * @brief _isMaster
     */
    bool _isMaster;

    /*!
     * @brief _started
     */
    std::atomic<bool> _started;

protected:
    virtual void execute() = 0;
    virtual void start() = 0;
    virtual void stop() = 0;
};
}
#endif
