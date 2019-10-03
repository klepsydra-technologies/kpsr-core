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

#ifndef EVENT_EMITTER_PUBLISHER_H
#define EVENT_EMITTER_PUBLISHER_H

#include <klepsydra/core/object_pool_publisher.h>
#include <klepsydra/core/event_emitter.h>

/**
*/
namespace kpsr
{
template<class T>
/*!
 * @brief The EventEmitterPublisher class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-test
 *
 * @details This publisher implementation is intended for testing purposes. It is synchronous and single-threaded. The goal is to use this publisher as a testing publisher for unit and integration testing. It derives from the ObjectPoolPublisher, which
 * allows to pre-allocate the message instances in a pool for increasing the performance.
 *
 */
class EventEmitterPublisher : public ObjectPoolPublisher<T>
{
public:
    /*!
     * @brief EventEmitterPublisher
     * @param container
     * @param eventName
     * @param eventEmitter
     * @param poolSize
     * @param initializerFunction
     * @param eventCloner
     */
    EventEmitterPublisher(Container * container,
                          std::string eventName,
                          EventEmitter & eventEmitter,
                          int poolSize,
                          std::function<void(T &)> initializerFunction,
                          std::function<void(const T &, T &)> eventCloner)
        : ObjectPoolPublisher<T>(container, eventName, "EVENT_EMITTER", poolSize, initializerFunction, eventCloner)
        , _eventEmitter(eventEmitter)
        , _eventName(eventName)
    {}

    /*!
     * @brief internalPublish publish events without copying.
     * @param event
     */
    void internalPublish(std::shared_ptr<const T> event) override {
        _eventEmitter.emitEvent(_eventName, 0, *event.get());
    }

private:
    EventEmitter & _eventEmitter;
    std::string _eventName;
};
}
#endif
