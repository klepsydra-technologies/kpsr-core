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

#ifndef EVENT_TRANSFORM_FORWARDER_H
#define EVENT_TRANSFORM_FORWARDER_H

#include <atomic>
#include <functional>

#include <klepsydra/core/publisher.h>

namespace kpsr {
template<class T, class U>
/*!
 * @brief The EventTransformForwarder class
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-application
 *
 * @details An optimised forwarding helper class. Its purpose is to optimise memory allocation and callstack when processing events,
 * transforming and forwarding them.
 */
class EventTransformForwarder
{
public:
    /*!
     * @brief EventTransformForwarder
     * @param transformFunction function that process and event and transforms it into another object.
     * @param destPublisher forwarding publisher.
     */
    EventTransformForwarder(std::function<void(const T &, U &)> transformFunction,
                            Publisher<U> *destPublisher)
        : forwarderListenerFunction(std::bind(&kpsr::EventTransformForwarder<T, U>::onEventReceived,
                                              this,
                                              std::placeholders::_1))
        , _transformFunction(transformFunction)
        , _destPublisher(destPublisher)
        , _eventReference(nullptr)
    {
        _process = [this](U &newEvent) { this->_transformFunction(*_eventReference, newEvent); };
    }

    /*!
     * @brief forwarderListenerFunction std::function to be added to the source subcriber.
     */
    std::function<void(const T &)> forwarderListenerFunction;

    /*!
     * \brief onEventReceived
     * \param event
     */
    void onEventReceived(const T &event)
    {
        _eventReference = &event;
        _destPublisher->processAndPublish(_process);
    }

private:
    std::function<void(const T &, U &)> _transformFunction;
    Publisher<U> *_destPublisher;
    std::atomic<const T *> _eventReference;
    std::function<void(U &)> _process;
};
} // namespace kpsr
#endif // EVENT_TRANSFORM_FORWARDER_H
