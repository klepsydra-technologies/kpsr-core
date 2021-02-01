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

#ifndef OBJECT_POOL_PUBLISHER_H
#define OBJECT_POOL_PUBLISHER_H

#include <spdlog/spdlog.h>


#include <klepsydra/core/publisher.h>
#include <klepsydra/core/smart_object_pool.h>

namespace kpsr
{
template <class T>
/*!
 * \brief The ObjectPoolPublisher class
 *
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-composition
 *
 * @details Abstract publisher based for most of the concrete implementations (ROS, DDS, ZMQ and event loop.).
 * It has an object pool that pre allocates objects before copying for publication.
 */
class ObjectPoolPublisher : public Publisher<T>
{
public:
    /*!
     * \brief ObjectPoolPublisher
     * \param container Container to attach in case of monitoring.
     * \param name name of the publisher
     * \param type valid values are EVENT_EMITTER, ROS, etc.
     * \param poolSize size of the pool to create.
     * \param initializerFunction init function to execute after creation.
     * \param eventCloner optional function to use instead of the copy.
     */
    ObjectPoolPublisher(Container * container,
                        const std::string & name,
                        const std::string type,
                        int poolSize,
                        std::function<void(T &)> initializerFunction,
                        std::function<void(const T &, T &)> eventCloner)
        : Publisher<T>(container, name, type)
        , _initializerFunction(initializerFunction)
        , _eventCloner(eventCloner)
    {
        _objectPool = poolSize == 0 ? nullptr : new SmartObjectPool<T>(poolSize, initializerFunction);
        this->_publicationStats._totalEventAllocations = poolSize;
    }

    virtual ~ObjectPoolPublisher() {
        if (_objectPool) {
            delete _objectPool;
        }
    }
    /*!
     * @brief internalPublish
     * @param eventData
     */
    void internalPublish(const T & eventData) override {
        if (_objectPool != nullptr) {
            try {
                std::shared_ptr<T> newEvent = std::move(_objectPool->acquire());
                if (_eventCloner == nullptr) {
                    (*newEvent) = eventData;
                } else {
                    _eventCloner(eventData, (*newEvent));
                }
                internalPublish(newEvent);
                return;
            } catch (std::out_of_range ex) {
                spdlog::info("ObjectPoolPublisher::internalPublish. Object Pool failure. {}", this->_publicationStats._name);
            }
        }
        this->_publicationStats._totalEventAllocations++;
        if (_eventCloner == nullptr) {
            std::shared_ptr<T> newEvent = std::make_shared<T>(eventData);
            internalPublish(newEvent);
            newEvent.reset();
        } else {
            std::shared_ptr<T> newEvent = std::make_shared<T>();
            if (_initializerFunction != nullptr) {
                _initializerFunction(* newEvent);
            }
            _eventCloner(eventData, (*newEvent));
            internalPublish(newEvent);
        }
    }

    /*!
     * @brief processAndPublish
     * @param process
     */
    void processAndPublish(std::function<void(T &)> process) {
        if (_objectPool != nullptr) {
            try {
                std::shared_ptr<T> newEvent = std::move(_objectPool->acquire());
                if (_initializerFunction != nullptr) {
                    _initializerFunction(*newEvent);
                }
                process(*newEvent);
                this->publish(newEvent);
                return;
            } catch (std::out_of_range ex) {
                spdlog::info("ObjectPoolPublisher::processAndPublish. Object Pool failure.");
            }
        }
        this->_publicationStats._totalEventAllocations++;
        std::shared_ptr<T> newEvent = std::make_shared<T>();
        if (_initializerFunction != nullptr) {
            _initializerFunction(*newEvent);
        }
        process(*newEvent);
        this->publish(newEvent);
    }

    /*!
     * @brief internalPublish abstract method that is implemented by the different concrete classes.
     * @param process
     */
    virtual void internalPublish(std::shared_ptr<const T> eventData) = 0;

private:
    std::function<void(T &)> _initializerFunction;
    SmartObjectPool<T> * _objectPool;
    std::function<void(const T &, T &)> _eventCloner;
};
}
#endif
