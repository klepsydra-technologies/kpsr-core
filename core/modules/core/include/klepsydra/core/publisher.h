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
    Publisher(Container * container, const std::string& name, const std::string& type)
        : _publicationStats(name, type) {
        if (container != nullptr) {
            container->attach(&this->_publicationStats);
        }
    }

    virtual ~Publisher() {}
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
