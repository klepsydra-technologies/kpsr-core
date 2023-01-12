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

#ifndef FROM_MIDDLEWARE_CHANNEL_H
#define FROM_MIDDLEWARE_CHANNEL_H

#include <klepsydra/core/event_transform_forwarder.h>
#include <klepsydra/serialization/mapper.h>

namespace kpsr {
template<class KpsrClass, class MddlwClass>
/*!
 * @brief The FromMiddlewareChannel class
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-composition
 *
 * @details Internal facility abstract class for reading data from middleware. Concrete implementations are available for ZMQ, ROS and DDS.
*/
class FromMiddlewareChannel
{
public:
    /*!
     * @brief FromMiddlewareChannel
     * @param internalPublisher
     */
    FromMiddlewareChannel(Publisher<KpsrClass> *internalPublisher)
        : _middlewareMapper()
        , _transformFunction(std::bind(&Mapper<KpsrClass, MddlwClass>::fromMiddleware,
                                       &_middlewareMapper,
                                       std::placeholders::_1,
                                       std::placeholders::_2))
        , _transformer(_transformFunction, internalPublisher)
    {}

    /*!
     * @brief onMiddlewareMessage
     * @param message
     */
    void onMiddlewareMessage(const MddlwClass &message) { _transformer.onEventReceived(message); }

private:
    Mapper<KpsrClass, MddlwClass> _middlewareMapper;
    std::function<void(const MddlwClass &, KpsrClass &)> _transformFunction;
    EventTransformForwarder<MddlwClass, KpsrClass> _transformer;
};
} // namespace kpsr

#endif // FROM_MIDDLEWARE_CHANNEL_H
