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

#ifndef TO_MIDDLEWARE_CHANNEL_H
#define TO_MIDDLEWARE_CHANNEL_H

#include <functional>
#include <exception>

#include <klepsydra/core/publisher.h>
#include <klepsydra/serialization/mapper.h>

namespace kpsr {
template<class KpsrClass, class MddlwClass>
/*!
 * @brief The ToMiddlewareChannel class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-composition
 *
 * @details Abstract class to transform and publish to the middleware.
 */
class ToMiddlewareChannel : public Publisher<KpsrClass>
{
public:
    /*!
     * @brief ToMiddlewareChannel
     * @param container
     * @param middlewarePublisher real native middleware publisher
     */
    ToMiddlewareChannel(Container * container, const std::string & name, Publisher<MddlwClass> * middlewarePublisher)
        : Publisher<KpsrClass>(container, name, "TO_MDLWR_CHANNEL")
        , _middlewarePublisher(middlewarePublisher)
        , _middlewareMapper()
        , _transformFunction(std::bind(&Mapper<KpsrClass, MddlwClass>::toMiddleware, &_middlewareMapper,
                                       std::placeholders::_1, std::placeholders::_2))
    {}

    /*!
     * @brief processAndPublish
     * @param process
     */
    void processAndPublish(std::function<void(KpsrClass &)> process) {
        std::shared_ptr<KpsrClass> newEvent = std::shared_ptr<KpsrClass>(new KpsrClass());
        this->_publicationStats._totalEventAllocations++;
        process(* newEvent.get());
        internalPublish(newEvent);
        return;
    }

protected:

    void internalPublish(const KpsrClass& event) {
        // This function can be optimised by creating in the constructor
        // and adding a pointer to the event to this class. However,
        // for thread-safety sake, I am keeping it for now.
        // benchmark results show an overhead of 3-4ns per call.
        std::function<void(MddlwClass &)> mapper = [&event, this] (MddlwClass & newEvent) {
            this->_transformFunction(event, newEvent);
        };

        _middlewarePublisher->processAndPublish(mapper);
    }

    void internalPublish(std::shared_ptr<const KpsrClass> event) {
        // This function can be optimised by creating in the constructor
        // and adding a pointer to the event to this class. However,
        // for thread-safety sake, I am keeping it for now.
        // benchmark results show an overhead of 3-4ns per call.
        std::function<void(MddlwClass &)> mapper = [&event, this] (MddlwClass & newEvent) {
            this->_transformFunction((*event.get()), newEvent);
        };

        _middlewarePublisher->processAndPublish(mapper);
    }

private:
    Publisher<MddlwClass> * _middlewarePublisher;
    Mapper<KpsrClass, MddlwClass> _middlewareMapper;
    std::function<void(const KpsrClass &, MddlwClass &)> _transformFunction;
};
}

#endif // TO_MIDDLEWARE_CHANNEL_H
