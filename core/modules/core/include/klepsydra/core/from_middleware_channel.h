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

#ifndef FROM_MIDDLEWARE_CHANNEL_H
#define FROM_MIDDLEWARE_CHANNEL_H

#include <klepsydra/core/event_transform_forwarder.h>
#include <klepsydra/serialization/mapper.h>
#include <klepsydra/serialization/identity_mapper.h>

namespace kpsr {
template<class KpsrClass, class MddlwClass>
/*!
 * @brief The FromMiddlewareChannel class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-composition
 *
 * @details Internal facility abstract class for reading data from middleware. Concrete implementations are available for ZMQ, ROS and DDS.
*/
class FromMiddlewareChannel {
public:
    /*!
     * @brief FromMiddlewareChannel
     * @param internalPublisher
     */
    FromMiddlewareChannel(Publisher<KpsrClass> * internalPublisher)
        : _middlewareMapper()
        , _transformFunction(std::bind(&Mapper<KpsrClass, MddlwClass>::fromMiddleware, &_middlewareMapper,
                                       std::placeholders::_1, std::placeholders::_2))
        , _transformer(_transformFunction, internalPublisher)
    {}

    /*!
     * @brief onMiddlewareMessage
     * @param message
     */
    void onMiddlewareMessage(const MddlwClass & message) {
        _transformer.onEventReceived(message);
    }

private:
    Mapper<KpsrClass, MddlwClass> _middlewareMapper;
    std::function<void(const MddlwClass &, KpsrClass &)> _transformFunction;
    EventTransformForwarder<MddlwClass, KpsrClass> _transformer;
};
}

#endif // FROM_MIDDLEWARE_CHANNEL_H
