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

#ifndef PUBLISHER_H
#define PUBLISHER_H

#include <memory>
#include <functional>

#include <klepsydra/core/publication_stats.h>
#include <klepsydra/core/container.h>

namespace kpsr
{
template <class T>
/*!
 * @brief The Publisher class
 *
 * @copyright Klepsydra Technologies 2019-2020.
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
    Publisher(Container * container, const std::string name, const std::string type)
        : _publicationStats(name, type) {
        if (container != nullptr) {
            container->attach(&this->_publicationStats);
        }
    }

    /*!
     * @brief publish
     * @param event
     */
    void publish(const T& event) {
        _publicationStats.startProcessMeassure();
        internalPublish(event);
        _publicationStats.stopProcessMeassure();
    }

    /*!
     * @brief publish without copy
     * @param event
     */
    void publish(std::shared_ptr<const T> event) {
        _publicationStats.startProcessMeassure();
        internalPublish(event);
        _publicationStats.stopProcessMeassure();
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
    virtual void internalPublish(const T& event) = 0;

    /*!
     * @brief internalPublish
     * @param event
      * Real implementation of the publishing method.
     */
    virtual void internalPublish(std::shared_ptr<const T> event) = 0;
};
}
#endif
