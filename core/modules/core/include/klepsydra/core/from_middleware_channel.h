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

#ifndef FROM_MIDDLEWARE_CHANNEL_H
#define FROM_MIDDLEWARE_CHANNEL_H

#include <klepsydra/core/event_transform_forwarder.h>
#include <klepsydra/serialization/mapper.h>

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
