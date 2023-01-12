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

#ifndef PUBLISHER_H
#define PUBLISHER_H

#include <functional>
#include <memory>

#include <klepsydra/core/container.h>
#include <klepsydra/core/publication_stats.h>

namespace kpsr {
template<class T>
/*!
 * @brief The Publisher class
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-application
 *
 * @details Main publisher API. Applications using Klepsydra should use pointers to this class in order to publish event to either other classes (intra process) or to the middleware (inter process)
*/
class Publisher
{
public:
    /*!
     * @brief Publisher
     * @param container
     * @param name
     * @param type
     */
    Publisher(Container *container, const std::string &name, const std::string &type)
        : _publicationStats(name, type)
    {
        if (container != nullptr) {
            container->attach(&this->_publicationStats);
        }
    }

    virtual ~Publisher() {}
    /*!
     * @brief publish
     * @param event
     */
    void publish(const T &event)
    {
        _publicationStats.startProcessMeasure();
        internalPublish(event);
        _publicationStats.stopProcessMeasure();
    }

    /*!
     * @brief publish without copy
     * @param event
     */
    void publish(std::shared_ptr<const T> event)
    {
        _publicationStats.startProcessMeasure();
        internalPublish(event);
        _publicationStats.stopProcessMeasure();
    }

    /*!
     * @brief processAndPublish
     * @param process
     */
    virtual void processAndPublish(std::function<void(T &)> process) = 0;

    /*!
     * @brief _publicationStats
     */
    PublicationStats _publicationStats;

protected:
    /*!
     * @brief internalPublish
     * @param event
      * Real implementation of the publishing method.
     */
    virtual void internalPublish(const T &event) = 0;

    /*!
     * @brief internalPublish
     * @param event
      * Real implementation of the publishing method.
     */
    virtual void internalPublish(std::shared_ptr<const T> event) = 0;
};
} // namespace kpsr
#endif
