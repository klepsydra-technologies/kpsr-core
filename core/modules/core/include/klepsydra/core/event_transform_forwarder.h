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

#ifndef EVENT_TRANSFORM_FORWARDER_H
#define EVENT_TRANSFORM_FORWARDER_H

#include <functional>
#include <atomic>

#include <klepsydra/core/publisher.h>

namespace kpsr
{
template <class T, class U>
/*!
 * @brief The EventTransformForwarder class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-application
 *
 * @details An optimised forwarding helper class. Its purpose is to optimise memory allocation and callstack when processing events,
 * transforming and forwarding them.
 */
class EventTransformForwarder {
public:
    /*!
     * @brief EventTransformForwarder
     * @param transformFunction function that process and event and transforms it into another object.
     * @param destPublisher forwarding publisher.
     */
    EventTransformForwarder(std::function<void(const T &, U &)> transformFunction,
                            Publisher<U> * destPublisher)
        : forwarderListenerFunction(std::bind(&kpsr::EventTransformForwarder<T, U>::onEventReceived, this, std::placeholders::_1))
        , _transformFunction(transformFunction)
        , _destPublisher(destPublisher)
        , _eventReference(nullptr)
    {
        _process = [this] (U & newEvent) {
            this->_transformFunction(* _eventReference, newEvent);
        };
    }

    /*!
     * @brief forwarderListenerFunction std::function to be added to the source subcriber.
     */
    std::function<void(const T &)> forwarderListenerFunction;

    /*!
     * \brief onEventReceived
     * \param event
     */
    void onEventReceived(const T & event) {
        _eventReference = &event;
        _destPublisher->processAndPublish(_process);
    }

private:
    std::function<void(const T &, U &)> _transformFunction;
    Publisher<U> * _destPublisher;
    std::atomic<const T *> _eventReference;
    std::function<void(U &)> _process;
};
}
#endif // EVENT_TRANSFORM_FORWARDER_H
