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

#ifndef TO_MIDDLEWARE_CHANNEL_H
#define TO_MIDDLEWARE_CHANNEL_H

#include <exception>
#include <functional>

#include <klepsydra/core/publisher.h>
#include <klepsydra/serialization/mapper.h>

namespace kpsr {
template<class KpsrClass, class MddlwClass>
/*!
 * @brief The ToMiddlewareChannel class
 *
 * @copyright 2023 Klepsydra Technologies AG
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
    ToMiddlewareChannel(Container *container,
                        const std::string &name,
                        Publisher<MddlwClass> *middlewarePublisher)
        : Publisher<KpsrClass>(container, name, "TO_MDLWR_CHANNEL")
        , _middlewarePublisher(middlewarePublisher)
        , _middlewareMapper()
        , _transformFunction(std::bind(&Mapper<KpsrClass, MddlwClass>::toMiddleware,
                                       &_middlewareMapper,
                                       std::placeholders::_1,
                                       std::placeholders::_2))
    {}

    /*!
     * @brief processAndPublish
     * @param process
     */
    void processAndPublish(std::function<void(KpsrClass &)> process)
    {
        std::shared_ptr<KpsrClass> newEvent = std::shared_ptr<KpsrClass>(new KpsrClass());
        this->_publicationStats._totalEventAllocations++;
        process(*newEvent.get());
        internalPublish(newEvent);
        return;
    }

protected:
    void internalPublish(const KpsrClass &event)
    {
        // This function can be optimised by creating in the constructor
        // and adding a pointer to the event to this class. However,
        // for thread-safety sake, I am keeping it for now.
        // benchmark results show an overhead of 3-4ns per call.
        std::function<void(MddlwClass &)> mapper = [&event, this](MddlwClass &newEvent) {
            this->_transformFunction(event, newEvent);
        };

        _middlewarePublisher->processAndPublish(mapper);
    }

    void internalPublish(std::shared_ptr<const KpsrClass> event)
    {
        // This function can be optimised by creating in the constructor
        // and adding a pointer to the event to this class. However,
        // for thread-safety sake, I am keeping it for now.
        // benchmark results show an overhead of 3-4ns per call.
        std::function<void(MddlwClass &)> mapper = [&event, this](MddlwClass &newEvent) {
            this->_transformFunction((*event.get()), newEvent);
        };

        _middlewarePublisher->processAndPublish(mapper);
    }

private:
    Publisher<MddlwClass> *_middlewarePublisher;
    Mapper<KpsrClass, MddlwClass> _middlewareMapper;
    std::function<void(const KpsrClass &, MddlwClass &)> _transformFunction;
};
} // namespace kpsr

#endif // TO_MIDDLEWARE_CHANNEL_H
