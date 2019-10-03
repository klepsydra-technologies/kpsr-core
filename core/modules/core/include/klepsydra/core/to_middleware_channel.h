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
    ToMiddlewareChannel(Container * container, std::string name, Publisher<MddlwClass> * middlewarePublisher)
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
